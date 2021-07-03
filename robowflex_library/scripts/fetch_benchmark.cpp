/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
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
    Benchmarker benchmark;

    // Create a motion planning request with a pose goal.
    auto request_1 = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request_1->setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request_1->setGoalConfiguration(fetch->getScratchState());

    // Setup three planners for benchmarking
    request_1->setConfig("RRTConnect");
    benchmark.addBenchmarkingRequest("rrtconnect", scene, planner, request_1);

    auto request_2 = request_1->clone();
    request_2->setConfig("RRT");
    benchmark.addBenchmarkingRequest("rrt", scene, planner, request_2);

    auto request_3 = request_1->clone();
    request_3->setConfig("RRTstar");
    benchmark.addBenchmarkingRequest("rrtstar", scene, planner, request_3);

    // Output results to an OMPL benchmarking file.
    Benchmarker::Options options;

    options.runs = 50;
    options.options =
        Benchmarker::WAYPOINTS | Benchmarker::CORRECT | Benchmarker::LENGTH | Benchmarker::SMOOTHNESS;

    // Benchmark and save results to "robowflex_fetch_benchmark/"
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_fetch_benchmark/")}, options);

    return 0;
}
