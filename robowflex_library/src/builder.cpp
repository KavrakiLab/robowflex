/* Author: Zachary Kingston */

#include <boost/math/constants/constants.hpp>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/builder.h>

using namespace robowflex;

const std::vector<std::string> MotionRequestBuilder::DEFAULT_CONFIGS({"CBiRRT2", "RRTConnect"});

MotionRequestBuilder::MotionRequestBuilder(const PlannerConstPtr &planner, const std::string &group_name)
  : planner_(planner)
  , robot_(planner->getRobot())
  , group_name_(group_name)
  , jmg_(robot_->getModelConst()->getJointModelGroup(group_name))
{
    request_.group_name = group_name_;

    // Default workspace
    moveit_msgs::WorkspaceParameters &wp = request_.workspace_parameters;
    wp.min_corner.x = wp.min_corner.y = wp.min_corner.z = -1;
    wp.max_corner.x = wp.max_corner.y = wp.max_corner.z = 1;

    // Default planning time
    request_.allowed_planning_time = 5.0;

    // Default planner (find an RRTConnect config, for Indigo)
    for (const auto &config : DEFAULT_CONFIGS)
        if (setConfig(config))
            break;
}

bool MotionRequestBuilder::setConfig(const std::string &requested_config)
{
    const auto &configs = planner_->getPlannerConfigs();

    std::vector<std::reference_wrapper<const std::string>> matches;
    for (const auto &config : configs)
    {
        if (config.find(requested_config) != std::string::npos)
            matches.emplace_back(config);
    }

    if (matches.empty())
        return false;

    const auto &found =
        std::min_element(matches.begin(), matches.end(),
                         [](const std::string &a, const std::string &b) { return a.size() < b.size(); });

    request_.planner_id = *found;
    ROS_INFO("Using planner: %s", request_.planner_id.c_str());
    return true;
}

void MotionRequestBuilder::setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp)
{
    request_.workspace_parameters = wp;
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState start_state(robot_->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(jmg_, joints);

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
}

void MotionRequestBuilder::setStartConfiguration(const robot_state::RobotStatePtr &state)
{
    moveit::core::robotStateToRobotStateMsg(*state, request_.start_state);
}

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotStatePtr state;
    state.reset(new robot_state::RobotState(robot_->getModelConst()));
    state->setJointGroupPositions(jmg_, joints);

    setGoalConfiguration(state);
}

void MotionRequestBuilder::setGoalConfiguration(const robot_state::RobotStatePtr &state)
{
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(*state, jmg_));
}

void MotionRequestBuilder::setGoalPose(const std::string &ee_name, const std::string &base_name,
                                       const RobotPose &pose, double tolerance)
{
    setGoalRegion(ee_name, base_name,                     //
                  pose, Geometry::makeSphere(tolerance),  //
                  Eigen::Quaterniond(pose.rotation()), {tolerance, tolerance, tolerance});
}

void MotionRequestBuilder::setGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const RobotPose &pose, const GeometryConstPtr &geometry,
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    moveit_msgs::Constraints constraints;

    constraints.position_constraints.push_back(TF::getPositionConstraint(ee_name, base_name, pose, geometry));
    constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));

    request_.goal_constraints.push_back(constraints);
}

void MotionRequestBuilder::addGoalRotaryTile(const std::string &ee_name, const std::string &base_name,
                                             const RobotPose &pose, const GeometryConstPtr &geometry,
                                             const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances, const RobotPose &offset,
                                             const Eigen::Vector3d &axis, unsigned int n)
{
    double pi2 = 2 * boost::math::constants::pi<double>();

    for (double angle = 0; angle < pi2; angle += pi2 / n)
    {
        Eigen::Quaterniond rotation(Eigen::AngleAxisd(angle, axis));
        RobotPose newPose = pose * rotation * offset;
        Eigen::Quaterniond newOrientation(rotation * orientation);

        setGoalRegion(ee_name, base_name, newPose, geometry, newOrientation, tolerances);
    }
}

void MotionRequestBuilder::addCylinderSideGrasp(const std::string &ee_name, const std::string &base_name,
                                                const RobotPose &pose, const GeometryConstPtr &cylinder,
                                                double distance, double depth, unsigned int n)
{
    // Grasping region to tile
    auto box = Geometry::makeBox(depth, depth, cylinder->getDimensions()[1]);
    RobotPose offset(Eigen::Translation3d(cylinder->getDimensions()[0] + distance, 0, 0));

    Eigen::Quaterniond orientation =
        Eigen::AngleAxisd(-boost::math::constants::pi<double>(), Eigen::Vector3d::UnitX())  //
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())                                    //
        * Eigen::AngleAxisd(boost::math::constants::pi<double>(), Eigen::Vector3d::UnitZ());

    addGoalRotaryTile(ee_name, base_name,                          //
                      pose, box, orientation, {0.01, 0.01, 0.01},  //
                      offset, Eigen::Vector3d::UnitZ(), n);
}

void MotionRequestBuilder::clearGoals()
{
    request_.goal_constraints.clear();
}

void MotionRequestBuilder::setPoseRegionTargets(
    const std::vector<geometry_msgs::PoseStamped> &pose_targets,
    const std::vector<moveit_msgs::GoalRegion> &pose_region_targets, const std::string &end_effector_link)
{
    request_.goal_constraints.clear();
    request_.goal_regions.clear();
    request_.goal_constraints.resize(pose_targets.size());
    request_.goal_regions.resize(pose_region_targets.size());

    for (unsigned int i = 0; i < pose_targets.size(); i++)
    {
        moveit_msgs::Constraints c =
            kinematic_constraints::constructGoalConstraints(end_effector_link, pose_targets[i]);
        request_.goal_constraints[i] =
            kinematic_constraints::mergeConstraints(request_.goal_constraints[i], c);
    }

    for (unsigned int i = 0; i < pose_region_targets.size(); i++)
        request_.goal_regions[i] = moveit_msgs::GoalRegion(pose_region_targets[i]);
}

void MotionRequestBuilder::setAllowedPlanningTime(double allowed_planning_time)
{
    request_.allowed_planning_time = allowed_planning_time;
}

void MotionRequestBuilder::addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                                 const RobotPose &pose,
                                                 const GeometryConstPtr &geometry,
                                                 const Eigen::Quaterniond &orientation,
                                                 const Eigen::Vector3d &tolerances)
{
    addPathPositionConstraint(ee_name, base_name, pose, geometry);
    addPathOrientationConstraint(ee_name, base_name, orientation, tolerances);
}

void MotionRequestBuilder::addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                     const RobotPose &pose,
                                                     const GeometryConstPtr &geometry)
{
    request_.path_constraints.position_constraints.push_back(
        TF::getPositionConstraint(ee_name, base_name, pose, geometry));
}

void MotionRequestBuilder::addPathOrientationConstraint(const std::string &ee_name,
                                                        const std::string &base_name,
                                                        const Eigen::Quaterniond &orientation,
                                                        const Eigen::Vector3d &tolerances)
{
    request_.path_constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));
}

moveit_msgs::Constraints &MotionRequestBuilder::getPathConstraints()
{
    return request_.path_constraints;
}

planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequest()
{
    return request_;
}

const planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequestConst() const
{
    return request_;
}

bool MotionRequestBuilder::toYAMLFile(const std::string &file)
{
    return IO::YAMLToFile(IO::toNode(request_), file);
}

bool MotionRequestBuilder::fromYAMLFile(const std::string &file)
{
    return IO::fromYAMLFile(request_, file);
}
