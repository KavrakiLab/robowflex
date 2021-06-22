/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_benchmark.cpp
 * A basic script that demonstrates benchmarking with the Fetch robot.
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 */

static const std::string GROUP = "arm_with_torso";

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
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("unfurl",  // Name of experiment
                          options,   // Options for internal profiler
                          5.0,       // Timeout allowed for ALL queries
                          50);       // Number of trials

    // Create a motion planning request with a pose goal.
    auto request_1 = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request_1->setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request_1->setGoalConfiguration(fetch->getScratchState());

    // Setup three planners for benchmarking
    request_1->setConfig("RRTConnect");
    experiment.addQuery("rrtconnect", scene, planner, request_1);

    auto request_2 = request_1->clone();
    request_2->setConfig("RRT");
    experiment.addQuery("rrt", scene, planner, request_2);

    auto request_3 = request_1->clone();
    request_3->setConfig("RRTstar");
    experiment.addQuery("rrtstar", scene, planner, request_3);

    auto dataset = experiment.benchmark(4);

    OMPLPlanDataSetOutputter output("robowflex_fetch_demo");
    output.dump(*dataset);

    return 0;
}
