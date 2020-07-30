/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

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
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch, "default");
    planner->initialize("package://fetch_moveit_config/config/ompl_planning.yaml");

    // Create a motion planning request with a pose goal.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request->setGoalConfiguration(fetch->getScratchState());

    request->setConfig("RRTstar");

    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("joint", scene, planner, request);

    Benchmarker::Options options;
    options.runs = 3;

    // Output results to an OMPL benchmarking file.
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_fetch_test/")}, options);

    return 0;
}
