#include "robowflex.h"

using namespace robowflex;

planning_interface::MotionPlanResponse PipelinePlanner::plan(Scene &scene,
                                                             const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    if (pipeline_)
        pipeline_->generatePlan(scene.getScene(), request, response);

    return response;
}

const std::vector<std::string> OMPLPlanner::DEFAULT_ADAPTERS(
    {"default_planner_request_adapters/AddTimeParameterization",  // Defaults for OMPL Pipeline
     "default_planner_request_adapters/FixWorkspaceBounds",       //
     "default_planner_request_adapters/FixStartStateBounds",      //
     "default_planner_request_adapters/FixStartStateCollision",   //
     "default_planner_request_adapters/FixStartStatePathConstraints"});

void OMPLPlanner::OMPLSettings::setParam(IO::Handler &handler) const
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

OMPLPlanner::OMPLPlanner(Robot &robot) : PipelinePlanner(robot)
{
}

bool OMPLPlanner::initialize(const std::string &config_file, const OMPLSettings settings, const std::string &plugin,
                             const std::vector<std::string> &adapters)
{
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

    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_.getModel(), handler_.getHandle(), "planning_plugin",
                                                            "request_adapters"));

    return true;
}
