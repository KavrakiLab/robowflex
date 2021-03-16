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
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>

using namespace robowflex;

// Typical name for RRTConnect configuration in MoveIt
const std::string MotionRequestBuilder::DEFAULT_CONFIG = "RRTConnectkConfigDefault";

MotionRequestBuilder::MotionRequestBuilder(const RobotConstPtr &robot) : robot_(robot)
{
    initialize();
}

MotionRequestBuilder::MotionRequestBuilder(const RobotConstPtr &robot, const std::string &group_name,
                                           const std::string &planner_config)
  : MotionRequestBuilder(robot)
{
    setPlanningGroup(group_name);

    if (not planner_config.empty())
        setConfig(planner_config);
}

MotionRequestBuilder::MotionRequestBuilder(const PlannerConstPtr &planner, const std::string &group_name,
                                           const std::string &planner_config)
  : MotionRequestBuilder(planner->getRobot())
{
    setPlanningGroup(group_name);
    setPlanner(planner);

    if (not planner_config.empty())
        setConfig(planner_config);
}

MotionRequestBuilder::MotionRequestBuilder(const MotionRequestBuilder &other)
  : MotionRequestBuilder(other.getRobot())
{
    request_ = other.getRequestConst();

    const auto &planner = other.getPlanner();
    if (planner)
        setPlanner(planner);
}

MotionRequestBuilderPtr MotionRequestBuilder::clone() const
{
    return std::make_shared<MotionRequestBuilder>(*this);
}

void MotionRequestBuilder::initialize()
{
    setConfig(DEFAULT_CONFIG);

    setWorkspaceBounds(Eigen::Vector3d::Constant(-1), Eigen::Vector3d::Constant(1));
    request_.allowed_planning_time = 5.0;
}

void MotionRequestBuilder::setPlanner(const PlannerConstPtr &planner)
{
    const auto &rname = robot_->getName();
    const auto &pname = planner->getRobot()->getName();

    if (rname != pname)
    {
        ROS_ERROR("Conflicting robots `%s` and `%s` in request builder!", rname.c_str(), pname.c_str());
        throw std::runtime_error("Invalid planner!");
    }

    planner_ = planner;
}

void MotionRequestBuilder::setPlanningGroup(const std::string &group_name)
{
    const auto &model = robot_->getModelConst();

    if (model->hasJointModelGroup(group_name))
    {
        group_name_ = group_name;
        jmg_ = robot_->getModelConst()->getJointModelGroup(group_name_);

        request_.group_name = group_name_;
    }
    else
    {
        ROS_ERROR("Joint group `%s` does not exist in robot!", group_name.c_str());
        throw std::runtime_error("Invalid joint group name!");
    }
}

bool MotionRequestBuilder::setConfig(const std::string &requested_config)
{
    if (not planner_)
    {
        ROS_INFO("No planner set! Using requested config `%s`", requested_config.c_str());
        request_.planner_id = requested_config;
        return true;
    }

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

    incrementVersion();

    request_.planner_id = *found;
    ROS_INFO("Requested Config: `%s`: Using planning config `%s`", requested_config.c_str(),
             request_.planner_id.c_str());
    return true;
}

void MotionRequestBuilder::setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp)
{
    incrementVersion();
    request_.workspace_parameters = wp;
}

void MotionRequestBuilder::setWorkspaceBounds(const Eigen::Ref<const Eigen::VectorXd> &min,
                                              const Eigen::Ref<const Eigen::VectorXd> &max)
{
    moveit_msgs::WorkspaceParameters wp;
    wp.min_corner.x = min[0];
    wp.min_corner.y = min[1];
    wp.min_corner.z = min[2];
    wp.max_corner.x = max[0];
    wp.max_corner.y = max[1];
    wp.max_corner.z = max[2];

    setWorkspaceBounds(wp);
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    if (not jmg_)
    {
        ROS_ERROR("No planning group set!");
        throw std::runtime_error("No planning group set!");
    }

    incrementVersion();

    robot_state::RobotState start_state(robot_->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(jmg_, joints);

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
}

void MotionRequestBuilder::setStartConfiguration(const robot_state::RobotState &state)
{
    incrementVersion();
    moveit::core::robotStateToRobotStateMsg(state, request_.start_state);
}

void MotionRequestBuilder::setStartConfiguration(const robot_state::RobotStatePtr &state)
{
    setStartConfiguration(*state);
}

void MotionRequestBuilder::useSceneStateAsStart(const SceneConstPtr &scene)
{
    setStartConfiguration(scene->getSceneConst()->getCurrentState());
}

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    if (not jmg_)
    {
        ROS_ERROR("No planning group set!");
        throw std::runtime_error("No planning group set!");
    }

    incrementVersion();

    robot_state::RobotStatePtr state;
    state.reset(new robot_state::RobotState(robot_->getModelConst()));
    state->setJointGroupPositions(jmg_, joints);

    setGoalConfiguration(state);
}

void MotionRequestBuilder::setGoalConfiguration(const robot_state::RobotStatePtr &state)
{
    if (not jmg_)
    {
        ROS_ERROR("No planning group set!");
        throw std::runtime_error("No planning group set!");
    }

    incrementVersion();
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
    incrementVersion();

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
    incrementVersion();
    request_.goal_constraints.clear();
}

void MotionRequestBuilder::setAllowedPlanningTime(double allowed_planning_time)
{
    incrementVersion();
    request_.allowed_planning_time = allowed_planning_time;
}

void MotionRequestBuilder::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
    incrementVersion();
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
    incrementVersion();
    request_.path_constraints.position_constraints.push_back(
        TF::getPositionConstraint(ee_name, base_name, pose, geometry));
}

void MotionRequestBuilder::addPathOrientationConstraint(const std::string &ee_name,
                                                        const std::string &base_name,
                                                        const Eigen::Quaterniond &orientation,
                                                        const Eigen::Vector3d &tolerances)
{
    incrementVersion();
    request_.path_constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));
}

moveit_msgs::Constraints &MotionRequestBuilder::getPathConstraints()
{
    incrementVersion();
    return request_.path_constraints;
}

planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequest()
{
    incrementVersion();
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
    incrementVersion();
    return IO::fromYAMLFile(request_, file);
}

const RobotConstPtr &MotionRequestBuilder::getRobot() const
{
    return robot_;
}

const PlannerConstPtr &MotionRequestBuilder::getPlanner() const
{
    return planner_;
}

const std::string &MotionRequestBuilder::getPlanningGroup() const
{
    return group_name_;
}

const std::string &MotionRequestBuilder::getPlannerConfig() const
{
    return request_.planner_id;
}
