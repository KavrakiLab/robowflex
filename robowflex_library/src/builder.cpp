/* Author: Zachary Kingston */

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <robowflex_library/constants.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/tf.h>

using namespace robowflex;

const std::vector<std::string> MotionRequestBuilder::DEFAULT_CONFIGS({"RRTConnect"});

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

MotionRequestBuilderPtr MotionRequestBuilder::clone() const
{
    auto clone = std::make_shared<MotionRequestBuilder>(planner_, group_name_);
    clone->getRequest() = request_;
    return clone;
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
    auto copy = pose;
    Eigen::Quaterniond orientation(pose.rotation());
    copy.linear() = Eigen::Matrix3d::Identity();
    setGoalRegion(ee_name, base_name,                     //
                  copy, Geometry::makeSphere(tolerance),  //
                  orientation, {tolerance, tolerance, tolerance});
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
    for (double angle = 0; angle < constants::two_pi; angle += constants::two_pi / n)
    {
        Eigen::Quaterniond rotation(Eigen::AngleAxisd(angle, axis));
        RobotPose new_pose = pose * rotation * offset;
        Eigen::Quaterniond new_orientation(rotation * orientation);

        setGoalRegion(ee_name, base_name, new_pose, geometry, new_orientation, tolerances);
    }
}

void MotionRequestBuilder::addCylinderSideGrasp(const std::string &ee_name, const std::string &base_name,
                                                const RobotPose &pose, const GeometryConstPtr &cylinder,
                                                double distance, double depth, unsigned int n)
{
    // Grasping region to tile
    auto box = Geometry::makeBox(depth, depth, cylinder->getDimensions()[1]);
    RobotPose offset(Eigen::Translation3d(cylinder->getDimensions()[0] + distance, 0, 0));

    Eigen::Quaterniond orientation = Eigen::AngleAxisd(-constants::pi, Eigen::Vector3d::UnitX())  //
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())             //
                                     * Eigen::AngleAxisd(constants::pi, Eigen::Vector3d::UnitZ());

    addGoalRotaryTile(ee_name, base_name,                          //
                      pose, box, orientation, {0.01, 0.01, 0.01},  //
                      offset, Eigen::Vector3d::UnitZ(), n);
}

void MotionRequestBuilder::clearGoals()
{
    request_.goal_constraints.clear();
}

void MotionRequestBuilder::setAllowedPlanningTime(double allowed_planning_time)
{
    request_.allowed_planning_time = allowed_planning_time;
}

void MotionRequestBuilder::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
    request_.num_planning_attempts = num_planning_attempts;
}

void MotionRequestBuilder::addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                                 const RobotPose &pose, const GeometryConstPtr &geometry,
                                                 const Eigen::Quaterniond &orientation,
                                                 const Eigen::Vector3d &tolerances)
{
    addPathPositionConstraint(ee_name, base_name, pose, geometry);
    addPathOrientationConstraint(ee_name, base_name, orientation, tolerances);
}

void MotionRequestBuilder::addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                     const RobotPose &pose, const GeometryConstPtr &geometry)
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

robot_state::RobotStatePtr MotionRequestBuilder::getStartConfiguration() const
{
    auto start_state = robot_->allocState();

    moveit::core::robotStateMsgToRobotState(request_.start_state, *start_state);
    start_state->update(true);
    return start_state;
}

robot_state::RobotStatePtr MotionRequestBuilder::getGoalConfiguration() const
{
    auto goal_state = robot_->allocState();

    if (request_.goal_constraints.size() != 1)
    {
        ROS_ERROR("Ambigous goal, %lu goal goal_constraints exist, returning default goal",
                  request_.goal_constraints.size());
        return goal_state;
    }

    if (request_.goal_constraints[0].joint_constraints.empty())
    {
        ROS_ERROR("No joint constraints specified, returning default goal");
        return goal_state;
    }

    std::map<std::string, double> variable_map;
    for (const auto &joint : request_.goal_constraints[0].joint_constraints)
        variable_map[joint.joint_name] = joint.position;

    // Start state includes attached objects and values for the non-group links.
    moveit::core::robotStateMsgToRobotState(request_.start_state, *goal_state);
    goal_state->setVariablePositions(variable_map);
    goal_state->update(true);

    return goal_state;
}

const planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequestConst() const
{
    return request_;
}

bool MotionRequestBuilder::toYAMLFile(const std::string &file) const
{
    return IO::YAMLToFile(IO::toNode(request_), file);
}

bool MotionRequestBuilder::fromYAMLFile(const std::string &file)
{
    return IO::fromYAMLFile(request_, file);
}
