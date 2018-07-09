/* Author: Zachary Kingston */

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

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
    const auto &configs = planner->getPlannerConfigs();
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
    robot_state::RobotState goal_state(robot_->getModelConst());
    goal_state.setJointGroupPositions(jmg_, joints);

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg_));
}

void MotionRequestBuilder::setGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const Eigen::Affine3d &pose, const GeometryConstPtr &geometry,
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    moveit_msgs::Constraints constraints;

    constraints.position_constraints.push_back(TF::getPositionConstraint(ee_name, base_name, pose, geometry));
    constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(constraints);
}

void MotionRequestBuilder::setAllowedPlanningTime(double allowed_planning_time)
{
    request_.allowed_planning_time = allowed_planning_time;
}

void MotionRequestBuilder::addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                                 const Eigen::Affine3d &pose,
                                                 const GeometryConstPtr &geometry,
                                                 const Eigen::Quaterniond &orientation,
                                                 const Eigen::Vector3d &tolerances)
{
    addPathPositionConstraint(ee_name, base_name, pose, geometry);
    addPathOrientationConstraint(ee_name, base_name, orientation, tolerances);
}

void MotionRequestBuilder::addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                     const Eigen::Affine3d &pose,
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

const planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequest() const
{
    return request_;
}

std::map<std::string, double>
robowflex::getFinalJointPositions(const planning_interface::MotionPlanResponse &response)
{
    moveit_msgs::MotionPlanResponse msg;
    response.getMessage(msg);

    const std::vector<double> &joint_positions = msg.trajectory.joint_trajectory.points.back().positions;
    const std::vector<std::string> &joint_names = msg.trajectory.joint_trajectory.joint_names;

    std::map<std::string, double> map;
    for (size_t i = 0; i < joint_names.size(); i++)
        map.emplace(joint_names[i], joint_positions[i]);

    return map;
}

bool MotionRequestBuilder::toYAMLFile(const std::string &file)
{
    return IO::YAMLtoFile(IO::toNode(request_), file);
}

bool MotionRequestBuilder::fromYAMLFile(const std::string &file)
{
    return IO::fromYAMLFile(request_, file);
}
