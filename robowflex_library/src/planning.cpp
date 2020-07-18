#include <robowflex_library/io.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>

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

void Planner::preRun(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
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

const std::vector<std::string> PoolPlanner::getPlannerConfigs() const
{
    return planners_.front()->getPlannerConfigs();
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

const std::vector<std::string> OMPL::OMPLPipelinePlanner::getPlannerConfigs() const
{
    return configs_;
}

///
/// Opt
///

bool Opt::loadConfig(IO::Handler &handler, const std::string &config_file)
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

    return true;
}

///
/// Opt::CHOMPPipelinePlanner
///

const std::string Opt::CHOMPPipelinePlanner::DEFAULT_PLUGIN("chomp_interface/CHOMPPlanner");
const std::vector<std::string>                                             //
    Opt::CHOMPPipelinePlanner::DEFAULT_ADAPTERS(                           //
        {"default_planner_request_adapters/FixWorkspaceBounds",            //
         "default_planner_request_adapters/FixStartStateBounds",           //
         "default_planner_request_adapters/FixStartStateCollision",        //
         "default_planner_request_adapters/FixStartStatePathConstraints",  //
         "default_planner_request_adapters/ResolveConstraintFrames",       //
         "default_planner_request_adapters/AddTimeParameterization"});

Opt::CHOMPPipelinePlanner::CHOMPPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : PipelinePlanner(robot, name)
{
}

bool Opt::CHOMPPipelinePlanner::initialize(const std::string &config_file, const std::string &plugin,
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

///
/// Opt::TrajOptPipelinePlanner
///

const std::string Opt::TrajOptPipelinePlanner::DEFAULT_PLUGIN("trajopt_interface/TrajOptPlanner");
const std::vector<std::string>                                        //
    Opt::TrajOptPipelinePlanner::DEFAULT_ADAPTERS(                    //
        {"default_planner_request_adapters/AddTimeParameterization",  //
         "default_planner_request_adapters/FixWorkspaceBounds",       //
         "default_planner_request_adapters/FixStartStateBounds",      //
         "default_planner_request_adapters/FixStartStateCollision",   //
         "default_planner_request_adapters/FixStartStatePathConstraints"});

Opt::TrajOptPipelinePlanner::TrajOptPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : PipelinePlanner(robot, name)
{
}

bool Opt::TrajOptPipelinePlanner::initialize(const std::string &config_file, const std::string &plugin,
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
