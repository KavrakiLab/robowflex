/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/detail/fetch.h>

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
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Sets the Fetch's base pose.
    fetch->setBasePose(1, 1, 0.5);

    // Sets the Fetch's head pose to look at a point.
    fetch->pointHead({2, 1, 1.5});

    // Create a motion planning request with a pose goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, GROUP));
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request->setGoalConfiguration(fetch->getScratchState());

    request->setConfig("PRM");

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("joint", scene, planner, request);

    // Output results to an OMPL benchmarking file.
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_fetch_test/")});

    return 0;
}
