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
static const double TIME = 60.0;

Profiler::ProgressCallback getGNUPlotCallback(IO::GNUPlotHelper &plotter, const std::string &field)
{
    return [&, field](const PlannerPtr &planner,                             //
                      const SceneConstPtr &scene,                            //
                      const planning_interface::MotionPlanRequest &request,  //
                      const Profiler::Result &result) {
        IO::GNUPlotHelper::TimeSeriesOptions tso;  // Plotting options for time series data
        tso.instance = field;
        tso.title = "Live Profiling";
        tso.x.label = "Time (s)";
        tso.x.min = 0.;
        tso.x.max = TIME;

        // Plot all progress collected so far.
        const auto &points = result.getProgressPropertiesAsPoints("time REAL", field);
        if (not points.empty())
        {
            tso.points.emplace(field, points);
            plotter.timeseries(tso);
        }
    };
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch);
    planner->initialize("package://robowflex_resources/fetch/config/ompl_planning.yaml");

    // Load an example problem of reaching into a box.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/fetch_box/scene0001.yaml");

    // Load the motion planning request and configure for profiling.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    request->fromYAMLFile("package://robowflex_library/yaml/fetch_box/request0001.yaml");
    request->setAllowedPlanningTime(TIME);
    request->setNumPlanningAttempts(1);
    request->setConfig("RRTstar");

    // Create the profiler. We will add some progress callbacks to plot progress properties while the planner
    // solves the problem.
    Profiler profiler;

    // Example of using plotting with the GNUPlot helper.
    IO::GNUPlotHelper gp;

    // Plotting options for time series data to display after planning is complete.
    IO::GNUPlotHelper::TimeSeriesOptions tso;
    tso.title = "RRT* Best Cost";
    tso.x.min = 0.;
    tso.x.max = TIME;

    // Add progress callbacks to plot progress data live while planning.
    profiler.addProgressCallback("plot cost", getGNUPlotCallback(gp, "best cost REAL"));
    profiler.addProgressCallback("plot iter", getGNUPlotCallback(gp, "iterations INTEGER"));

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;

    // 5 iterations of profiling the same problem
    for (std::size_t i = 0; i < 5; ++i)
    {
        Profiler::Result result;
        if (profiler.profilePlan(planner, scene, request->getRequest(), options, result))
        {
            tso.points.emplace(log::format("Trial %1%", i + 1),  //
                               result.getProgressPropertiesAsPoints("time REAL", "best cost REAL"));

            // Plot all progress collected so far.
            gp.timeseries(tso);
        }
    }

    RBX_INFO("Press Enter to Exit...");
    std::cin.ignore();

    return 0;
}
