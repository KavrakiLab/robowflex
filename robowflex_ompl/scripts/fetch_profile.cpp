/* Author: Zachary Kingston */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io/gnuplot.h>

#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

/* \file fetch_profile.cpp
 * A basic script that demonstrates using the plan profiler with the Fetch robot.
 * The plan profiler is a tool to instrument a single planning run and extract relevant progress properties or
 * metrics. For larger scale planner testing, check out the benchmarking tools.
 * This uses IO::GNUPlotHelper, which helps with live visualization of data.
 */

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/fetch_box/scene0001.yaml");

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch);
    planner->initialize("package://robowflex_resources/fetch/config/ompl_planning.yaml");

    // Create a motion planning request with a pose goal.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    request->fromYAMLFile("package://robowflex_library/yaml/fetch_box/request0001.yaml");
    request->setAllowedPlanningTime(30.0);
    request->setNumPlanningAttempts(1);
    request->setConfig("RRTstar");

    Profiler profiler;

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;

    // Example of using plotting with the GNUPlot helper
    IO::GNUPlotHelper gp;

    IO::GNUPlotHelper::TimeSeriesOptions tso;  // Plotting options for time series data
    tso.title = "RRT* Best Cost";

    // 5 iterations of profiling the same problem
    for (std::size_t i = 0; i < 5; ++i)
    {
        Profiler::Result result;
        bool success = profiler.profilePlan(planner, scene, request->getRequest(), options, result);

        // Extract cost progress property for plotting.
        if (success)
        {
            tso.points.emplace(log::format("Cost %1%", i),  //
                               result.getProgressPropertiesAsPoints("time REAL", "best cost REAL"));

            // Plot all progress collected so far.
            gp.timeseries(tso);
        }
    }

    RBX_INFO("Press Enter to Exit...");
    std::cin.ignore();

    return 0;
}
