/* Author: Zachary Kingston */

#include <moveit/robot_state/conversions.h>

#include <robowflex_library/io.h>
#include <robowflex_library/log.h>
#include <robowflex_library/macros.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>

#define ROBOWFLEX_HAS_CARTESIAN_INTERPOLATOR                                                                 \
    ROBOWFLEX_INCLUDE_EXISTS("moveit/robot_state/cartesian_interpolator.h")

#if ROBOWFLEX_HAS_CARTESIAN_INTERPOLATOR
#include <moveit/robot_state/cartesian_interpolator.h>
#endif

using namespace robowflex;

///
/// Planner
///

Planner::Planner(const RobotPtr &robot, const std::string &name)
  : robot_(robot), handler_(robot_->getHandler(), name), name_(name)
{
}

const RobotPtr Planner::getRobot() const
{
    return robot_;
}

const std::string &Planner::getName() const
{
    return name_;
}

void Planner::preRun(const SceneConstPtr & /*scene*/,
                     const planning_interface::MotionPlanRequest & /*request*/)
{
}

std::map<std::string, Planner::ProgressProperty> Planner::getProgressProperties(
    const SceneConstPtr & /*scene*/, const planning_interface::MotionPlanRequest & /*request*/) const

{
    return {};
}

///
/// PoolPlanner
///

PoolPlanner::PoolPlanner(const RobotPtr &robot, unsigned int n, const std::string &name)
  : Planner(robot, name), pool_(n)
{
}

std::shared_ptr<Pool::Job<planning_interface::MotionPlanResponse>>
PoolPlanner::submit(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    return pool_.submit(make_function([&] {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return !planners_.empty(); });

        auto planner = planners_.front();
        planners_.pop();

        lock.unlock();

        auto result = planner->plan(scene, request);

        lock.lock();
        planners_.emplace(planner);
        cv_.notify_one();

        return result;
    }));
}

planning_interface::MotionPlanResponse PoolPlanner::plan(const SceneConstPtr &scene,
                                                         const planning_interface::MotionPlanRequest &request)
{
    auto job = submit(scene, request);
    return job->get();
}

std::vector<std::string> PoolPlanner::getPlannerConfigs() const
{
    return planners_.front()->getPlannerConfigs();
}

///
/// SimpleCartesianPlanner
///

SimpleCartesianPlanner::SimpleCartesianPlanner(const RobotPtr &robot, const std::string &name)
  : Planner(robot, name)
{
}

planning_interface::MotionPlanResponse
SimpleCartesianPlanner::plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse temp;
    if (request.goal_constraints.size() > 1)
    {
        RBX_ERROR("SimpleCartesianPlanner only supports queries with a single goal!");
        temp.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return temp;
    }

    const auto &goal = request.goal_constraints[0];
    if (not goal.joint_constraints.empty() or not goal.visibility_constraints.empty())
    {
        RBX_ERROR("SimpleCartesianPlanner only supports pose goals!");
        temp.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return temp;
    }

    if (goal.position_constraints.size() != 1 and goal.orientation_constraints.size() != 1)
    {
        RBX_ERROR("SimpleCartesianPlanner requires single position and orientation constraint!");
        temp.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return temp;
    }

    // Get sampleable region through IK Query
    const auto &pc = goal.position_constraints[0];
    const auto &oc = goal.orientation_constraints[0];
    Robot::IKQuery query(request.group_name, pc, oc);
    query.attempts = request.num_planning_attempts;
    query.timeout = request.allowed_planning_time;
    query.scene = scene;

    // Get starting state
    auto state = robot_->allocState();
    moveit::core::robotStateMsgToRobotState(request.start_state, *state);

    return plan(*state, query);
}

planning_interface::MotionPlanResponse SimpleCartesianPlanner::plan(const robot_state::RobotState &start,
                                                                    const Robot::IKQuery &request)
{
    planning_interface::MotionPlanResponse response;
    if (request.tips.size() > 1)
    {
        RBX_ERROR("SimpleCartesianPlanner only supports queries with a single goal!");
        response.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return response;
    }

    std::string tip = request.tips[0];
    if (tip.empty())
    {
        const auto &tips = robot_->getSolverTipFrames(request.group);
        if (tips.size() == 1)
            tip = tips[0];
        else
        {
            RBX_ERROR("Request needs tip frame name!");
            response.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
            return response;
        }
    }

    // Get JMG, link model, and GSVCF
    const auto &model = robot_->getModelConst();
    const auto &jmg = model->getJointModelGroup(request.group);
    const auto &lm = model->getLinkModel(tip);

    moveit::core::GroupStateValidityCallbackFn gsvcf;
    if (request.scene)
        gsvcf = request.scene->getGSVCF(false);

    std::vector<robot_state::RobotStatePtr> traj;

    moveit::core::MaxEEFStep step(max_step_pos_, max_step_rot_);
    moveit::core::JumpThreshold jump(jump_threshold_rev_, jump_threshold_pri_);

    double time = 0;
    bool success = false;
    ros::WallTime start_time = ros::WallTime::now();

    for (std::size_t i = 0;                                             //
         not success                                                    //
         and ((request.attempts) ? i < request.attempts : true)         //
         and ((request.timeout > 0.) ? time < request.timeout : true);  //
         ++i)
    {
        auto state = start;

        RobotPose pose;
        request.sampleRegion(pose, 0);

#if ROBOWFLEX_HAS_CARTESIAN_INTERPOLATOR
        double percentage =                                             //
            moveit::core::CartesianInterpolator::computeCartesianPath(  //
                &state, jmg, traj, lm, pose, true, step, jump, gsvcf);
#else
        double percentage =              //
            state.computeCartesianPath(  //
                jmg, traj, lm, pose, true, step, jump, gsvcf);
#endif

        // Check if successful, output is percent of path computed.
        success = std::fabs(percentage - 1.) < constants::eps;
        time = (ros::WallTime::now() - start_time).toSec();
    }

    Trajectory output(robot_, request.group);
    for (const auto &state : traj)
        output.addSuffixWaypoint(*state);

    output.computeTimeParameterization();

    response.trajectory_ = output.getTrajectory();
    response.planning_time_ = time;

    if (success)
        response.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
        response.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

    return response;
}

void SimpleCartesianPlanner::setMaxStep(double position, double rotation)
{
    max_step_pos_ = position;
    max_step_rot_ = rotation;
}

void SimpleCartesianPlanner::setJumpThreshold(double prismatic, double revolute)
{
    jump_threshold_pri_ = prismatic;
    jump_threshold_rev_ = revolute;
}

std::vector<std::string> SimpleCartesianPlanner::getPlannerConfigs() const
{
    return std::vector<std::string>{"cartesian"};
}

///
/// PipelinePlanner
///

PipelinePlanner::PipelinePlanner(const RobotPtr &robot, const std::string &name) : Planner(robot, name)
{
}

planning_interface::MotionPlanResponse
PipelinePlanner::plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    if (pipeline_)
        pipeline_->generatePlan(scene->getSceneConst(), request, response);

    return response;
}

///
/// OMPL
///

bool OMPL::loadOMPLConfig(IO::Handler &handler, const std::string &config_file,
                          std::vector<std::string> &configs)
{
    if (config_file.empty())
        return false;

    const auto &config = IO::loadFileToYAML(config_file);
    if (!config.first)
    {
        RBX_ERROR("Failed to load planner configs.");
        return false;
    }

    handler.loadYAMLtoROS(config.second);

    const auto &planner_configs = config.second["planner_configs"];
    if (planner_configs)
    {
        for (YAML::const_iterator it = planner_configs.begin(); it != planner_configs.end(); ++it)
            configs.push_back(it->first.as<std::string>());
    }

    return true;
}

///
/// OMPL::Settings
///

OMPL::Settings::Settings()
  : max_goal_samples(10)
  , max_goal_sampling_attempts(1000)
  , max_planning_threads(4)
  , max_solution_segment_length(0.0)
  , max_state_sampling_attempts(4)
  , minimum_waypoint_count(10)
  , simplify_solutions(true)
  , hybridize_solutions(true)
  , interpolate_solutions(true)
  , use_constraints_approximations(false)
  , display_random_valid_states(false)
  , link_for_exploration_tree("")
  , maximum_waypoint_distance(0.0)
{
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

///
/// OMPL::PipelinePlanner
///

const std::string OMPL::OMPLPipelinePlanner::DEFAULT_PLUGIN("ompl_interface/OMPLPlanner");
const std::vector<std::string>                                        //
    OMPL::OMPLPipelinePlanner::DEFAULT_ADAPTERS(                      //
        {"default_planner_request_adapters/AddTimeParameterization",  //
         "default_planner_request_adapters/FixWorkspaceBounds",       //
         "default_planner_request_adapters/FixStartStateBounds",      //
         "default_planner_request_adapters/FixStartStateCollision",   //
         "default_planner_request_adapters/FixStartStatePathConstraints"});

OMPL::OMPLPipelinePlanner::OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : PipelinePlanner(robot, name)
{
}

bool OMPL::OMPLPipelinePlanner::initialize(const std::string &config_file, const OMPL::Settings &settings,
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

    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_->getModelConst(), handler_.getHandle(),
                                                            "planning_plugin", "request_adapters"));

    return true;
}

std::vector<std::string> OMPL::OMPLPipelinePlanner::getPlannerConfigs() const
{
    return configs_;
}

///
/// Opt
///

bool opt::loadConfig(IO::Handler &handler, const std::string &config_file)
{
    if (config_file.empty())
        return false;

    const auto &config = IO::loadFileToYAML(config_file);
    if (!config.first)
    {
        RBX_ERROR("Failed to load planner configs.");
        return false;
    }

    handler.loadYAMLtoROS(config.second);

    return true;
}

///
/// opt::CHOMPSettings
///

opt::CHOMPSettings::CHOMPSettings()
  : planning_time_limit(10.0)
  , max_iterations(200)
  , max_iterations_after_collision_free(5)
  , smoothness_cost_weight(0.1)
  , obstacle_cost_weight(0.0)
  , learning_rate(0.01)
  , animate_path(true)
  , add_randomness(false)
  , smoothness_cost_velocity(0.0)
  , smoothness_cost_acceleration(1.0)
  , smoothness_cost_jerk(0.0)
  , hmc_discretization(0.01)
  , hmc_stochasticity(0.01)
  , hmc_annealing_factor(0.99)
  , use_hamiltonian_monte_carlo(false)
  , ridge_factor(0.001)
  , use_pseudo_inverse(false)
  , pseudo_inverse_ridge_factor(1e-4)
  , animate_endeffector(false)
  , animate_endeffector_segment("")
  , joint_update_limit(0.1)
  , collision_clearence(0.2)
  , collision_threshold(0.07)
  , random_jump_amount(1.0)
  , use_stochastic_descent(true)
  , enable_failure_recovery(true)
  , max_recovery_attempts(5)
  , trajectory_initialization_method("quintic-spline")
  , start_state_max_bounds_error(0.1)
{
}

void opt::CHOMPSettings::setParam(IO::Handler &handler) const
{
    const std::string prefix;
    handler.setParam(prefix + "planning_time_limit", planning_time_limit);
    handler.setParam(prefix + "max_iterations", max_iterations);
    handler.setParam(prefix + "max_iterations_after_collision_free", max_iterations_after_collision_free);
    handler.setParam(prefix + "smoothness_cost_weight", smoothness_cost_weight);
    handler.setParam(prefix + "obstacle_cost_weight", obstacle_cost_weight);
    handler.setParam(prefix + "learning_rate", learning_rate);
    handler.setParam(prefix + "animate_path", animate_path);
    handler.setParam(prefix + "add_randomness", add_randomness);
    handler.setParam(prefix + "smoothness_cost_velocity", smoothness_cost_velocity);
    handler.setParam(prefix + "smoothness_cost_acceleration", smoothness_cost_acceleration);
    handler.setParam(prefix + "smoothness_cost_jerk", smoothness_cost_jerk);
    handler.setParam(prefix + "hmc_discretization", hmc_discretization);
    handler.setParam(prefix + "hmc_stochasticity", hmc_stochasticity);
    handler.setParam(prefix + "hmc_annealing_factor", hmc_annealing_factor);
    handler.setParam(prefix + "use_hamiltonian_monte_carlo", use_hamiltonian_monte_carlo);
    handler.setParam(prefix + "ridge_factor", ridge_factor);
    handler.setParam(prefix + "use_pseudo_inverse", use_pseudo_inverse);
    handler.setParam(prefix + "pseudo_inverse_ridge_factor", pseudo_inverse_ridge_factor);
    handler.setParam(prefix + "animate_endeffector", animate_endeffector);
    handler.setParam(prefix + "animate_endeffector_segment", animate_endeffector_segment);
    handler.setParam(prefix + "joint_update_limit", joint_update_limit);
    handler.setParam(prefix + "collision_clearence", collision_clearence);
    handler.setParam(prefix + "collision_threshold", collision_threshold);
    handler.setParam(prefix + "random_jump_amount", random_jump_amount);
    handler.setParam(prefix + "use_stochastic_descent", use_stochastic_descent);
    handler.setParam(prefix + "enable_failure_recovery", enable_failure_recovery);
    handler.setParam(prefix + "max_recovery_attempts", max_recovery_attempts);
    handler.setParam(prefix + "trajectory_initialization_method", trajectory_initialization_method);
    handler.setParam(prefix + "start_state_max_bounds_error", start_state_max_bounds_error);
}

///
/// opt::CHOMPPipelinePlanner
///

const std::string opt::CHOMPPipelinePlanner::DEFAULT_PLUGIN("chomp_interface/CHOMPPlanner");
const std::vector<std::string>                                             //
    opt::CHOMPPipelinePlanner::DEFAULT_ADAPTERS(                           //
        {"default_planner_request_adapters/FixWorkspaceBounds",            //
         "default_planner_request_adapters/FixStartStateBounds",           //
         "default_planner_request_adapters/FixStartStateCollision",        //
         "default_planner_request_adapters/FixStartStatePathConstraints",  //
         // "default_planner_request_adapters/ResolveConstraintFrames",       //
         "default_planner_request_adapters/AddTimeParameterization"});

opt::CHOMPPipelinePlanner::CHOMPPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : PipelinePlanner(robot, name)
{
}

bool opt::CHOMPPipelinePlanner::initialize(const CHOMPSettings &settings, const std::string &plugin,
                                           const std::vector<std::string> &adapters)
{
    settings.setParam(handler_);
    return finishInitialize(plugin, adapters);
}

bool opt::CHOMPPipelinePlanner::initialize(const std::string &config_file, const std::string &plugin,
                                           const std::vector<std::string> &adapters)
{
    if (!loadConfig(handler_, config_file))
        return false;

    return finishInitialize(plugin, adapters);
}

bool opt::CHOMPPipelinePlanner::finishInitialize(const std::string &plugin,
                                                 const std::vector<std::string> &adapters)
{
    configs_.emplace_back("chomp");
    handler_.setParam("planning_plugin", plugin);

    std::stringstream ss;
    for (std::size_t i = 0; i < adapters.size(); ++i)
    {
        ss << adapters[i];
        if (i < adapters.size() - 1)
            ss << " ";
    }

    handler_.setParam("request_adapters", ss.str());
    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_->getModelConst(), handler_.getHandle(),
                                                            "planning_plugin", "request_adapters"));

    return true;
}

std::vector<std::string> opt::CHOMPPipelinePlanner::getPlannerConfigs() const
{
    return configs_;
}

///
/// opt::TrajOptPipelinePlanner
///

const std::string opt::TrajOptPipelinePlanner::DEFAULT_PLUGIN("trajopt_interface/TrajOptPlanner");
const std::vector<std::string>                                        //
    opt::TrajOptPipelinePlanner::DEFAULT_ADAPTERS(                    //
        {"default_planner_request_adapters/AddTimeParameterization",  //
         "default_planner_request_adapters/FixWorkspaceBounds",       //
         "default_planner_request_adapters/FixStartStateBounds",      //
         "default_planner_request_adapters/FixStartStateCollision",   //
         "default_planner_request_adapters/FixStartStatePathConstraints"});

opt::TrajOptPipelinePlanner::TrajOptPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : PipelinePlanner(robot, name)
{
}

bool opt::TrajOptPipelinePlanner::initialize(const std::string &config_file, const std::string &plugin,
                                             const std::vector<std::string> &adapters)
{
    if (!loadConfig(handler_, config_file))
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

    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_->getModelConst(), handler_.getHandle(),
                                                            "planning_plugin", "request_adapters"));

    return true;
}
