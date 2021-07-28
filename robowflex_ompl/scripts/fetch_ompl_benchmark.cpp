/* Author: Zachary Kingston */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

Profiler::ComputeMetricCallback getNumVerticesCallback()
{
    return [](const PlannerPtr &planner,                             //
              const SceneConstPtr &scene,                            //
              const planning_interface::MotionPlanRequest &request,  //
              const PlanData &run) -> PlannerMetric {
        const auto &ompl_planner = std::dynamic_pointer_cast<const OMPL::OMPLInterfacePlanner>(planner);
        const auto &op = ompl_planner->getLastSimpleSetup()->getPlanner();

        ompl::base::PlannerData pd(op->getSpaceInformation());
        op->getPlannerData(pd);

        return (int)pd.numVertices();
    };
}

Profiler::ComputeMetricCallback getGoalDistanceCallback()
{
    return [](const PlannerPtr &planner,                             //
              const SceneConstPtr &scene,                            //
              const planning_interface::MotionPlanRequest &request,  //
              const PlanData &run) -> PlannerMetric {
        const auto &ompl_planner = std::dynamic_pointer_cast<const OMPL::OMPLInterfacePlanner>(planner);

        const auto &pdef = ompl_planner->getLastSimpleSetup()->getProblemDefinition();
        double distance = pdef->getSolutionDifference();

        if (distance == -1)
        {
            const auto &start = pdef->getStartState(0);
            const auto &goal = std::dynamic_pointer_cast<ompl::base::GoalRegion>(pdef->getGoal());
            distance = goal->distanceGoal(start);
        }

        return distance;
    };
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch, "default");
    planner->initialize("package://robowflex_resources/fetch/config/ompl_planning.yaml");

    // Create a motion planning request with a pose goal.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request->setGoalConfiguration(fetch->getScratchState());

    request->setConfig("RRTstar");

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("unfurl",  // Name of experiment
                          options,   // Options for internal profiler
                          10.0,      // Timeout allowed for ALL queries
                          5);        // Number of trials

    auto &profiler = experiment.getProfiler();
    profiler.addMetricCallback("goal_distance", getGoalDistanceCallback());
    profiler.addMetricCallback("num_vertices", getNumVerticesCallback());

    experiment.addQuery("rrtstar", scene, planner, request);

    // Note: Only 1 thread can be used when profiling the OMPL planners, as planning contexts under the hood
    // are reused between queries.
    auto dataset = experiment.benchmark(1);

    OMPLPlanDataSetOutputter output("robowflex_fetch_ompl");
    output.dump(*dataset);

    return 0;
}
