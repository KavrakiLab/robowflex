#include <moveit/ompl_interface/model_based_planning_context.h>

#include <robowflex_library/macros.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>

#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

OMPL::OMPLInterfacePlanner::OMPLInterfacePlanner(const RobotPtr &robot, const std::string &name)
  : Planner(robot, name)
{
}

bool OMPL::OMPLInterfacePlanner::initialize(const std::string &config_file, const OMPL::Settings settings)
{
    if (!loadOMPLConfig(handler_, config_file, configs_))
        return false;

    interface_.reset(new ompl_interface::OMPLInterface(robot_->getModel(), handler_.getHandle()));

    settings.setParam(handler_);

    interface_->simplifySolutions(settings.simplify_solutions);
    hybridize_ = settings.hybridize_solutions;
    interpolate_ = settings.interpolate_solutions;

    auto &pcm = interface_->getPlanningContextManager();
    pcm.setMaximumSolutionSegmentLength(settings.maximum_waypoint_distance);
    pcm.setMinimumWaypointCount(settings.minimum_waypoint_count);

    pcm.setMaximumPlanningThreads(settings.max_planning_threads);
    pcm.setMaximumGoalSamples(settings.max_goal_samples);
    pcm.setMaximumStateSamplingAttempts(settings.max_state_sampling_attempts);
    pcm.setMaximumGoalSamplingAttempts(settings.max_goal_sampling_attempts);

    if (settings.max_solution_segment_length > std::numeric_limits<double>::epsilon())
        pcm.setMaximumSolutionSegmentLength(settings.max_solution_segment_length);

    return true;
}

ompl_interface::ModelBasedPlanningContextPtr OMPL::OMPLInterfacePlanner::getPlanningContext(
    const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) const
{
    return interface_->getPlanningContext(scene->getSceneConst(), request);
}

void OMPL::OMPLInterfacePlanner::preRun(const SceneConstPtr &scene,
                                        const planning_interface::MotionPlanRequest &request)
{
    refreshContext(scene, request, true);
}

planning_interface::MotionPlanResponse OMPL::OMPLInterfacePlanner::plan(
    const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    refreshContext(scene, request);
    if (not ss_)
        return response;

    if (pre_plan_callback_)
        pre_plan_callback_(context_, scene, request);

    context_->solve(response);
    return response;
}

std::map<std::string, Planner::ProgressProperty> OMPL::OMPLInterfacePlanner::getProgressProperties(
    const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) const
{
    refreshContext(scene, request);
    if (not ss_)
        return {};

    const auto &planner = ss_->getPlanner();

#if ROBOWFLEX_AT_LEAST_KINETIC
    return planner->getPlannerProgressProperties();

    // As in Indigo they are boost::function
#else
    std::map<std::string, Planner::ProgressProperty> ret;
    for (const auto &pair : planner->getPlannerProgressProperties())
    {
        auto function = pair.second;
        ret[pair.first] = [function] { return function(); };
    }

    return ret;
#endif
}

void OMPL::OMPLInterfacePlanner::refreshContext(const SceneConstPtr &scene,
                                                const planning_interface::MotionPlanRequest &request,
                                                bool force) const
{
    const auto &scene_id = scene->getKey();
    const auto &request_hash = IO::getMessageMD5(request);

    bool same_scene = compareIDs(scene_id, last_scene_id_);
    bool same_request = request_hash == last_request_hash_;

    if (not force and ss_ and same_scene and same_request)
    {
        RBX_INFO("Reusing Cached Context!");
        return;
    }

    context_ = getPlanningContext(scene, request);
    if (not context_)
    {
        RBX_ERROR("Context was not set!");
        ss_ = nullptr;
        return;
    }

#if ROBOWFLEX_AT_LEAST_MELODIC
    context_->setInterpolation(interpolate_);
    context_->setHybridize(hybridize_);
#endif

    ss_ = context_->getOMPLSimpleSetup();

    last_scene_id_ = scene_id;
    last_request_hash_ = request_hash;

    RBX_INFO("Refreshed Context!");
}

ompl::geometric::SimpleSetupPtr OMPL::OMPLInterfacePlanner::getLastSimpleSetup() const
{
    return ss_;
}

std::vector<std::string> OMPL::OMPLInterfacePlanner::getPlannerConfigs() const
{
    return configs_;
}

ompl_interface::OMPLInterface &OMPL::OMPLInterfacePlanner::getInterface() const
{
    if (!interface_)
    {
        RBX_WARN("Interface is not initialized before call to OMPLInterfacePlanner::initialize.");
    }
    return *interface_;
}

void OMPL::OMPLInterfacePlanner::setPrePlanCallback(const PrePlanCallback &prePlanCallback)
{
    pre_plan_callback_ = prePlanCallback;
}
