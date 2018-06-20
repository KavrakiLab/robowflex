#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <robowflex_library/robowflex.h>

using namespace robowflex;

const std::vector<std::string> MotionRequestBuilder::DEFAULT_CONFIGS = {"CBiRRT2", "RRTConnect"};

MotionRequestBuilder::MotionRequestBuilder(const Planner &planner, const std::string &group_name)
  : planner_(planner)
  , robot_(planner.getRobot())
  , group_name_(group_name)
  , jmg_(robot_.getModel()->getJointModelGroup(group_name))
{
    request_.group_name = group_name_;

    // Default workspace
    moveit_msgs::WorkspaceParameters &wp = request_.workspace_parameters;
    wp.min_corner.x = wp.min_corner.y = wp.min_corner.z = -1;
    wp.max_corner.x = wp.max_corner.y = wp.max_corner.z = 1;

    // Default planning time
    request_.allowed_planning_time = 5.0;

    // Default planner (find an RRTConnect config, for Indigo)
    const auto &configs = planner.getPlannerConfigs();
    for (const auto &config : DEFAULT_CONFIGS)
        if (setConfig(config))
            break;
}

bool MotionRequestBuilder::setConfig(const std::string &requested_config)
{
    const auto &configs = planner_.getPlannerConfigs();

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
    return true;
}

void MotionRequestBuilder::setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp)
{
    request_.workspace_parameters = wp;
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState start_state(robot_.getModel());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(jmg_, joints);

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
}

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState goal_state(robot_.getModel());
    goal_state.setJointGroupPositions(jmg_, joints);

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg_));
}

void MotionRequestBuilder::setGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const Eigen::Affine3d &pose, const Geometry &geometry,
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

void MotionRequestBuilder::addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                                 const Eigen::Affine3d &pose, const Geometry &geometry,
                                                 const Eigen::Quaterniond &orientation,
                                                 const Eigen::Vector3d &tolerances)
{
    addPathPositionConstraint(ee_name, base_name, pose, geometry);
    addPathOrientationConstraint(ee_name, base_name, orientation, tolerances);
}

void MotionRequestBuilder::addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                     const Eigen::Affine3d &pose, const Geometry &geometry)
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

std::vector<double> robowflex::getFinalJointPositions(planning_interface::MotionPlanResponse response)
{
    moveit_msgs::MotionPlanResponse msg;
    response.getMessage(msg);
    const std::vector<double> &joint_positions = msg.trajectory.joint_trajectory.points.back().positions;
    // request.setStartConfiguration(joint_positions);
    return joint_positions;
}

bool MotionRequestBuilder::toYAMLFile(const std::string &file)
{
    return IO::messageToYAMLFile(request_, file);
}

bool MotionRequestBuilder::fromYAMLFile(const std::string &file)
{
    return IO::YAMLFileToMessage(request_, file);
}

planning_interface::MotionPlanResponse
PipelinePlanner::plan(const Scene &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    if (pipeline_)
        pipeline_->generatePlan(scene.getSceneConst(), request, response);

    return response;
}

void OMPL::Settings::setParam(IO::Handler &handler) const
{
    const std::string prefix = "ompl/";
    handler.setParam(prefix + "max_goal_samples", max_goal_samples);
    handler.setParam(prefix + "max_goal_sampling_attempts", max_goal_sampling_attempts);
    handler.setParam(prefix + "max_planning_threads", max_planning_threads);
    handler.setParam(prefix + "max_solution_segment_length", max_solution_segment_length);
    handler.setParam(prefix + "max_state_sampling_attempts", max_state_sampling_attempts);
    handler.setParam(prefix + "minimum_waypoint_count", minimum_waypoint_count);
    handler.setParam(prefix + "simplify_solutions", simplify_solutions);
    handler.setParam(prefix + "use_constraints_approximations", use_constraints_approximations);
    handler.setParam(prefix + "display_random_valid_states", display_random_valid_states);
    handler.setParam(prefix + "link_for_exploration_tree", link_for_exploration_tree);
    handler.setParam(prefix + "maximum_waypoint_distance", maximum_waypoint_distance);
}

const std::string OMPL::OMPLPipelinePlanner::DEFAULT_PLUGIN({"ompl_interface/OMPLPlanner"});
const std::vector<std::string>  //
    OMPL::OMPLPipelinePlanner::DEFAULT_ADAPTERS(
        {"default_planner_request_adapters/AddTimeParameterization",  //
         "default_planner_request_adapters/FixWorkspaceBounds",       //
         "default_planner_request_adapters/FixStartStateBounds",      //
         "default_planner_request_adapters/FixStartStateCollision",   //
         "default_planner_request_adapters/FixStartStatePathConstraints"});

namespace
{
    bool loadOMPLConfig(IO::Handler &handler, const std::string &config_file,
                        std::vector<std::string> &configs)
    {
        if (config_file.empty())
            return false;

        auto &config = IO::loadFileToYAML(config_file);
        if (!config.first)
        {
            ROS_ERROR("Failed to load planner configs.");
            return false;
        }

        handler.loadYAMLtoROS(config.second);

        auto &planner_configs = config.second["planner_configs"];
        if (planner_configs)
        {
            for (YAML::const_iterator it = planner_configs.begin(); it != planner_configs.end(); ++it)
                configs.push_back(it->first.as<std::string>());
        }

        return true;
    }
}  // namespace

OMPL::OMPLPipelinePlanner::OMPLPipelinePlanner(Robot &robot) : PipelinePlanner(robot)
{
}

bool OMPL::OMPLPipelinePlanner::initialize(const std::string &config_file, const OMPL::Settings settings,
                                           const std::string &plugin,
                                           const std::vector<std::string> &adapters)
{
    if (!loadOMPLConfig(handler_, config_file, configs_))
        return false;

    handler_.setParam("planning_plugin", plugin);

    std::stringstream ss;
    for (std::size_t i = 0; i < adapters.size(); ++i)
    {
        ss << adapters[i];
        if (i < adapters.size() - 1)
            ss << " ";
    }

    handler_.setParam("request_adapters", ss.str());
    settings.setParam(handler_);

    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_.getModel(), handler_.getHandle(),
                                                            "planning_plugin", "request_adapters"));

    return true;
}

const std::vector<std::string> OMPL::OMPLPipelinePlanner::getPlannerConfigs() const
{
    return configs_;
}

// OMPL::OMPLInterfacePlanner::OMPLInterfacePlanner(Robot &robot)
//   : Planner(robot), interface_(robot.getModel(), robot.getHandler().getHandle())
// {
// }

// bool OMPL::OMPLInterfacePlanner::initialize(const std::string &config_file, const OMPL::Settings settings)
// {
//     if (!loadOMPLConfig(handler_, config_file, configs_))
//         return false;

//     settings.setParam(handler_);
//     return true;
// }

// planning_interface::MotionPlanResponse
// OMPL::OMPLInterfacePlanner::plan(Scene &scene, const planning_interface::MotionPlanRequest &request)
// {
//     planning_interface::MotionPlanResponse response;
//     response.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

//     ompl_interface::ModelBasedPlanningContextPtr context = interface_.getPlanningContext(scene.getScene(),
//     request);

//     if (!context)
//         return response;

//     context->clear();
//     bool result = context->solve(response);

//     return response;
// }

// const std::vector<std::string> OMPL::OMPLInterfacePlanner::getPlannerConfigs() const
// {
//     return configs_;
// }
