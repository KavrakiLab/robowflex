/* Author: Zachary Kingston */

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <robowflex_library/builder.h>
#include <robowflex_library/constants.h>
#include <robowflex_library/random.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>

using namespace robowflex;

// Typical name for RRTConnect configuration in MoveIt
const std::string MotionRequestBuilder::DEFAULT_CONFIG = "RRTConnectkConfigDefault";

MotionRequestBuilder::MotionRequestBuilder(const RobotConstPtr &robot) : robot_(robot)
{
    initialize();

    robot_state::RobotState start_state(robot_->getModelConst());
    start_state.setToDefaultValues();

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
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

    setWorkspaceBounds(Eigen::Vector3d::Constant(-constants::default_workspace_bound),
                       Eigen::Vector3d::Constant(constants::default_workspace_bound));
    request_.allowed_planning_time = constants::default_allowed_planning_time;
}

void MotionRequestBuilder::setPlanner(const PlannerConstPtr &planner)
{
    const auto &rname = robot_->getName();
    const auto &pname = planner->getRobot()->getName();

    if (rname != pname)
    {
        RBX_ERROR("Conflicting robots `%s` and `%s` in request builder!", rname, pname);
        throw Exception(1, "Invalid planner!");
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
        RBX_ERROR("Joint group `%s` does not exist in robot!", group_name);
        throw Exception(1, "Invalid joint group name!");
    }
}

bool MotionRequestBuilder::setConfig(const std::string &requested_config)
{
    if (not planner_)
    {
        RBX_INFO("No planner set! Using requested config `%s`", requested_config);
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
    RBX_INFO("Requested Config: `%s`: Using planning config `%s`", requested_config, request_.planner_id);
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

bool MotionRequestBuilder::swapStartWithGoal()
{
    if (request_.goal_constraints.size() != 1)
    {
        RBX_ERROR("Multiple goal constraints exist, cannot swap start with goal");
        return false;
    }

    if (request_.goal_constraints[0].joint_constraints.empty())
    {
        RBX_ERROR("No joint goal is specified, cannot swap start with goal");
        return false;
    }

    const auto &start = getStartConfiguration();
    const auto &goal = getGoalConfiguration();
    clearGoals();

    setStartConfiguration(goal);
    setGoalConfiguration(start);
    return true;
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    incrementVersion();
    robot_->setStateMsgGroupState(request_.start_state, request_.group_name, joints);
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
    setStartConfiguration(scene->getCurrentStateConst());
}

bool MotionRequestBuilder::attachObjectToStart(ScenePtr scene, const std::string &object)
{
    // Attach object to current start configuration.
    const auto &start = getStartConfiguration();
    if (not scene->attachObject(*start, object))
        return false;

    useSceneStateAsStart(scene);
    return true;
}

bool MotionRequestBuilder::attachObjectToStartConst(const SceneConstPtr &scene, const std::string &object)
{
    auto copy = scene->deepCopy();
    return attachObjectToStart(copy, object);
}

void MotionRequestBuilder::addGoalConfiguration(const std::vector<double> &joints)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    incrementVersion();

    robot_state::RobotStatePtr state;
    state.reset(new robot_state::RobotState(robot_->getModelConst()));
    state->setJointGroupPositions(jmg_, joints);

    addGoalConfiguration(state);
}

void MotionRequestBuilder::addGoalConfiguration(const robot_state::RobotStatePtr &state)
{
    addGoalConfiguration(*state);
}

void MotionRequestBuilder::addGoalConfiguration(const robot_state::RobotState &state)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    incrementVersion();
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(state, jmg_));
}

void MotionRequestBuilder::addGoalFromIKQuery(const Robot::IKQuery &query)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    if (group_name_ != query.group)
    {
        RBX_ERROR("Planning group in IK query `%1%` not the same as request `%2%`", query.group, group_name_);
        throw Exception(1, "Mismatched query groups!");
    }

    if (query.regions.size() > 1)
    {
        RBX_ERROR("Cannot set goal request from IK query with multiple targets!");
        throw Exception(1, "Tried to set goal from multi-target request!");
    }

    std::string tip_to_use = query.tips[0];
    if (tip_to_use.empty())
    {
        const auto &tips = robot_->getSolverTipFrames(group_name_);
        if (tips.empty() or tips.size() > 1)
        {
            RBX_ERROR("Unable to find tip frame for request.");
            throw Exception(1, "Unable to find tip frame for request.");
        }

        tip_to_use = tips[0];
    }

    const std::string &base = robot_->getSolverBaseFrame(group_name_);
    if (base.empty())
    {
        RBX_ERROR("Failed to get base frame for request.");
        throw Exception(1, "Unable to find base frame for request.");
    }

    addGoalRegion(tip_to_use, base, query.region_poses[0], query.regions[0], query.orientations[0],
                  query.tolerances[0]);
}

void MotionRequestBuilder::addGoalPose(const std::string &ee_name, const std::string &base_name,
                                       const RobotPose &pose, double tolerance)
{
    auto copy = pose;
    Eigen::Quaterniond orientation(pose.rotation());
    copy.linear() = Eigen::Matrix3d::Identity();
    addGoalRegion(ee_name, base_name,                     //
                  copy, Geometry::makeSphere(tolerance),  //
                  orientation, {tolerance, tolerance, tolerance});
}

void MotionRequestBuilder::addGoalRegion(const std::string &ee_name, const std::string &base_name,
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

        addGoalRegion(ee_name, base_name, new_pose, geometry, new_orientation, tolerances);
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

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    clearGoals();
    addGoalConfiguration(joints);
}

void MotionRequestBuilder::setGoalConfiguration(const robot_state::RobotStatePtr &state)
{
    clearGoals();
    addGoalConfiguration(state);
}

void MotionRequestBuilder::setGoalConfiguration(const robot_state::RobotState &state)
{
    clearGoals();
    addGoalConfiguration(state);
}

void MotionRequestBuilder::setGoalFromIKQuery(const Robot::IKQuery &query)
{
    clearGoals();
    addGoalFromIKQuery(query);
}

void MotionRequestBuilder::setGoalPose(const std::string &ee_name, const std::string &base_name,
                                       const RobotPose &pose, double tolerance)
{
    clearGoals();
    addGoalPose(ee_name, base_name, pose, tolerance);
}

void MotionRequestBuilder::setGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const RobotPose &pose, const GeometryConstPtr &geometry,
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    clearGoals();
    addGoalRegion(ee_name, base_name, pose, geometry, orientation, tolerances);
}

void MotionRequestBuilder::precomputeGoalConfigurations(std::size_t n_samples, const ScenePtr &scene,
                                                        const ConfigurationValidityCallback &callback)
{
    // Allocate samplers for each region
    constraint_samplers::ConstraintSamplerManager manager;
    std::vector<constraint_samplers::ConstraintSamplerPtr> samplers;
    for (const auto &goal : request_.goal_constraints)
    {
        samplers.emplace_back(manager.selectSampler(scene->getSceneConst(), group_name_, goal));
        samplers.back()->setGroupStateValidityCallback(scene->getGSVCF(false));
    }

    clearGoals();

    // Clone start
    robot_state::RobotState state = *robot_->getScratchStateConst();

    // Sample n_samples to add to new request
    std::size_t n = n_samples;
    while (n)
    {
        auto sampler = RNG::uniformSample(samplers);
        if (sampler->sample(state) and (not callback or callback(state)))
        {
            addGoalConfiguration(state);
            n--;
        }
    }
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
        RBX_ERROR("Ambiguous goal, %lu goal goal_constraints exist, returning default goal",
                  request_.goal_constraints.size());
        return goal_state;
    }

    if (request_.goal_constraints[0].joint_constraints.empty())
    {
        RBX_ERROR("No joint constraints specified, returning default goal");
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

    bool success = IO::fromYAMLFile(request_, file);
    setPlanningGroup(request_.group_name);
    return success;
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
