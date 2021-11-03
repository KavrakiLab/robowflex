/* Author: Zachary Kingston */

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

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
    const auto &si = ss_->getSpaceInformation();

    // Set desired default optimization objective
    ompl::base::OptimizationObjectivePtr obj;
    switch (objective_)
    {
        case MAX_MIN_CLEARANCE:
        {
            auto pl_obj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
            auto cl_obj = std::make_shared<ompl::base::MaximizeMinClearanceObjective>(si);

            auto multi_obj = std::make_shared<ompl::base::MultiOptimizationObjective>(si);
            multi_obj->addObjective(pl_obj, 1. - clearance_objective_weight_);
            multi_obj->addObjective(cl_obj, clearance_objective_weight_);
            multi_obj->lock();

            obj = multi_obj;
            break;
        }
        case PATH_LENGTH:
        default:
            obj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
            break;
    }

    // Set optimization objective in problem definition
    auto pdef = ss_->getProblemDefinition();
    pdef->setOptimizationObjective(obj);

    // Set objective in path simplifier
    if (use_objective_simplifier_)
        ss_->getPathSimplifier() =
            std::make_shared<ompl::geometric::PathSimplifier>(si, pdef->getGoal(), obj);

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
        RBX_WARN("Interface is not initialized before call to OMPLInterfacePlanner::initialize.");

    return *interface_;
}

void OMPL::OMPLInterfacePlanner::setPrePlanCallback(const PrePlanCallback &prePlanCallback)
{
    pre_plan_callback_ = prePlanCallback;
}

void OMPL::OMPLInterfacePlanner::setHybridize(bool hybridize)
{
    hybridize_ = hybridize;
}

void OMPL::OMPLInterfacePlanner::setInterpolate(bool interpolate)
{
    interpolate_ = interpolate;
}

void OMPL::OMPLInterfacePlanner::usePathLengthObjective()
{
    objective_ = PATH_LENGTH;
}

void OMPL::OMPLInterfacePlanner::setObjectiveSimplifier(bool use_objective)
{
    use_objective_simplifier_ = use_objective;
}

void OMPL::OMPLInterfacePlanner::useMaxMinClearanceObjective(double weight)
{
    if (weight > 1. or weight < 0.)
        throw std::runtime_error("Weight must be [0, 1]");

    objective_ = MAX_MIN_CLEARANCE;
    clearance_objective_weight_ = weight;
}
