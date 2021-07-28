/* Author: Zachary Kingston */

#include <boost/program_options.hpp>

#include <ompl/base/Planner.h>

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io/gnuplot.h>
#include <robowflex_library/io/visualization.h>

#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;
namespace po = boost::program_options;

/* \file fetch_profile.cpp
 * A basic script that demonstrates using the plan profiler with the Fetch robot.
 * The plan profiler is a tool to instrument a single planning run and extract relevant progress properties or
 * metrics. For larger scale planner testing, check out the benchmarking tools.
 * This uses IO::GNUPlotHelper, which helps with live visualization of data. Make sure GNUPlot is installed.
 */

static const std::string GROUP = "arm_with_torso";
static const double TIME = 60.0;

/** \brief Creates a progress callback function to plot the progress property \a field live using
 * GNUPlot.
 */
Profiler::ProgressCallback getGNUPlotCallback(IO::GNUPlotHelper &plotter, const std::string &field)
{
    return [&, field](const PlannerPtr &planner,                             //
                      const SceneConstPtr &scene,                            //
                      const planning_interface::MotionPlanRequest &request,  //
                      const PlanData &result) {
        IO::GNUPlotHelper::TimeSeriesOptions tso;  // Plotting options for time series data
        tso.instance = field;
        tso.title = "Live Profiling";
        tso.x.label = "Time (s)";
        tso.x.min = 0.;
        tso.x.max = TIME;

        // Plot all progress collected so far.
        const auto &points = result.getProgressPropertiesAsPoints("time REAL", field);
        tso.points.emplace(field, points);
        plotter.timeseries(tso);
    };
}

/** \brief Get a custom progress property function allocator that extracts the planner data from the
 * underlying OMPL motion planner.
 */
Profiler::ProgressPropertyAllocator getNumVerticesAllocator()
{
    return [](const PlannerPtr &planner,   //
              const SceneConstPtr &scene,  //
              const planning_interface::MotionPlanRequest &request) -> Planner::ProgressProperty {
        // Extract OMPL level information
        const auto &ompl_planner = std::dynamic_pointer_cast<const OMPL::OMPLInterfacePlanner>(planner);
        const auto &ss = ompl_planner->getLastSimpleSetup();
        const auto &op = ss->getPlanner();

        return [op] {
            ompl::base::PlannerData pd(op->getSpaceInformation());
            op->getPlannerData(pd);

            return std::to_string(pd.numVertices());
        };
    };
}

/** \brief Get a custom progress callback allocator that allocates a function to visualize the current
 * planning graph of the OMPL planner.
 */
Profiler::ProgressCallbackAllocator getRVIZGraphVisualizationAllocator(IO::RVIZHelperPtr &rviz)
{
    return [rviz](const PlannerPtr &planner,   //
                  const SceneConstPtr &scene,  //
                  const planning_interface::MotionPlanRequest &request) -> Profiler::ProgressCallback {
        // Extract OMPL level information
        const auto &ompl_planner = std::dynamic_pointer_cast<const OMPL::OMPLInterfacePlanner>(planner);
        const auto &ss = ompl_planner->getLastSimpleSetup();
        const auto &op = ss->getPlanner();

        return [rviz, op](const PlannerPtr &planner,                             //
                          const SceneConstPtr &scene,                            //
                          const planning_interface::MotionPlanRequest &request,  //
                          const PlanData &result) {
            const auto &robot = planner->getRobot();
            const auto &ss = op->getSpaceInformation()->getStateSpace();

            // Get the planner data
            ompl::base::PlannerData pd(op->getSpaceInformation());
            op->getPlannerData(pd);

            // Compute the pose of the end-effector for all new vertices
            std::vector<double> reals;

            RobotPoseVector poses(pd.numVertices());
            for (std::size_t i = 0; i < pd.numVertices(); ++i)
            {
                const auto &pdv = pd.getVertex(i);
                const auto &state = pdv.getState();
                ss->copyToReals(reals, state);
                robot->setGroupState(GROUP, reals);

                poses[i] = robot->getLinkTF("wrist_roll_link");
            }

            // Color map for lines in graph. Vertices have tags used by some planners.
            const std::vector<Eigen::Vector4d> color_map = {{0, 0, 1, 0.4},  //
                                                            {0, 1, 0, 0.4},  //
                                                            {1, 0.37, 0.81, 0.4}};

            std::vector<Eigen::Vector3d> points;  // Points in the line
            std::vector<Eigen::Vector4d> colors;  // Color of the points in the line

            // Get the points used in the graph
            std::vector<unsigned int> edges;
            for (std::size_t i = 0; i < pd.numVertices(); ++i)
            {
                pd.getEdges(i, edges);
                const auto &src = pd.getVertex(i);
                for (const auto &edge : edges)
                {
                    const auto &dst = pd.getVertex(edge);

                    std::size_t si = src.getTag();
                    points.emplace_back(poses[i].translation());
                    colors.emplace_back((si < color_map.size()) ? color_map[si] : color_map[0]);

                    std::size_t di = dst.getTag();
                    points.emplace_back(poses[edge].translation());
                    colors.emplace_back((di < color_map.size()) ? color_map[di] : color_map[0]);
                }
            }

            // Update markers on RViz
            rviz->removeAllMarkers();
            rviz->addLineMarker("graph", points, colors, 0.005);
            rviz->updateMarkers();
        };
    };
}

/** \brief Get a custom metric that computes the distance to the goal for the best solution the planner found.
 */
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

    // Program Arguments
    bool gnuplot;
    bool live_plotting;
    bool custom_progress;
    bool rviz_enable;
    double progress_rate;
    std::string planner_name;

    po::options_description description;
    description.add_options()                                                                            //
        ("help,h", "Produces this help message")                                                         //
        ("planner,p", po::value<std::string>(&planner_name)->default_value("RRTstar"),                   //
         "Which OMPL Planner to use.")                                                                   //
        ("rate,r", po::value<double>(&progress_rate)->default_value(0.5),                                //
         "Update rate of planner progress property collection in seconds.")                              //
        ("gnuplot,g", po::bool_switch(&gnuplot),                                                         //
         "Enables GNUPlot visualization of the best cost path (if a progress property)")                 //
        ("liveplot,l", po::bool_switch(&live_plotting),                                                  //
         "Enables live GNUPlot plotting of `best cost` and `num vertices` (if enabled and available).")  //
        ("rviz,v", po::bool_switch(&rviz_enable),                                                        //
         "Enables live visualization of the planning graph through the RViz MarkerArray.")               //
        ("vertices,n", po::bool_switch(&custom_progress),                                                //
         "Enables the number of vertices in the planning graph with a custom progress property.")        //
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);

    if (vm.count("help"))
    {
        std::cout << description << std::endl;
        return false;
    }

    po::notify(vm);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch);

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    planner->initialize("package://robowflex_resources/fetch/config/ompl_planning.yaml", settings);

    // Load an example problem of reaching into a box.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/fetch_box/scene0001.yaml");

    // Load the motion planning request and configure for profiling.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    request->fromYAMLFile("package://robowflex_library/yaml/fetch_box/request0001.yaml");
    request->setAllowedPlanningTime(TIME);
    request->setNumPlanningAttempts(1);
    request->setConfig(planner_name);

    scene->getCurrentState() = *request->getStartConfiguration();

    // Create the profiler. We will add some progress callbacks to plot progress properties while the planner
    // solves the problem.
    Profiler profiler;

    // Example of using plotting with the GNUPlot helper.
    IO::GNUPlotHelper gp;

    // Setup RViz
    auto rviz = std::make_shared<IO::RVIZHelper>(fetch);
    if (rviz_enable)
        rviz->updateScene(scene);

    // Plotting options for time series data to display after planning is complete.
    IO::GNUPlotHelper::TimeSeriesOptions tso;
    tso.title = "Best Cost";
    tso.x.min = 0.;
    tso.x.max = TIME;

    // Add a custom metric
    profiler.addProgressAllocator("num vertices INTEGER", getNumVerticesAllocator());

    // Add progress callbacks to plot progress data live while planning.
    if (live_plotting)
    {
        profiler.addProgressCallback(getGNUPlotCallback(gp, "best cost REAL"));
        profiler.addProgressCallback(getGNUPlotCallback(gp, "num vertices INTEGER"));
    }

    // Add a callback to visualize the planning graph in RViz.
    if (rviz_enable)
        profiler.addProgressCallbackAllocator(getRVIZGraphVisualizationAllocator(rviz));

    // Add a custom metric to compute the distance to go to the goal.
    profiler.addMetricCallback("goal_distance", getGoalDistanceCallback());

    // Profiler options.
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    options.progress_update_rate = progress_rate;

    // 5 iterations of profiling the same problem
    for (std::size_t i = 0; i < 5; ++i)
    {
        PlanData result;
        if (profiler.profilePlan(planner, scene, request->getRequest(), options, result))
        {
            if (gnuplot)
            {
                tso.points.emplace(log::format("Trial %1%", i + 1),  //
                                   result.getProgressPropertiesAsPoints("time REAL", "best cost REAL"));

                // Plot all progress collected so far.
                gp.timeseries(tso);
            }

            if (rviz_enable)
                rviz->updateTrajectory(*result.trajectory);
        }

        RBX_INFO("Press Enter to Continue...");
        std::cin.ignore();
    }

    RBX_INFO("Press Enter to Exit...");
    std::cin.ignore();

    return 0;
}
