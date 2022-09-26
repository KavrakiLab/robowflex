/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

#include <robowflex_library/io/gnuplot.h>

using namespace robowflex;

/* \file fetch_benchmark.cpp
 * A basic script that demonstrates benchmarking with the Fetch robot.
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 * Note: This script requires GNUPlot for live visualization of timing data.
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
                          100);      // Number of trials

    // Create a motion planning request with a pose goal.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request->setGoalConfiguration(fetch->getScratchState());

    request->setConfig("RRTConnect");
    experiment.addQuery("rrtconnect", scene, planner, request->getRequest());

    request->setConfig("RRT");
    experiment.addQuery("rrt", scene, planner, request->getRequest());

    request->setConfig("PRM");
    experiment.addQuery("prm", scene, planner, request->getRequest());

    request->setConfig("KPIECE");
    experiment.addQuery("kpiece", scene, planner, request->getRequest());

    request->setConfig("BKPIECE");
    experiment.addQuery("bkpiece", scene, planner, request->getRequest());

    request->setConfig("LBKPIECE");
    experiment.addQuery("lbkpiece", scene, planner, request->getRequest());

    request->setConfig("EST");
    experiment.addQuery("est", scene, planner, request->getRequest());

    // Use the post-query callback to visualize the data live.
    IO::GNUPlotPlanDataSetOutputter plot("time");
    experiment.setPostQueryCallback(
        [&](PlanDataSetPtr dataset, const PlanningQuery &) { plot.dump(*dataset); });

    auto dataset = experiment.benchmark(4);

    OMPLPlanDataSetOutputter output("robowflex_fetch_demo");
    output.dump(*dataset);

    return 0;
}
