#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_tesseract/tesseract_planners.h>
#include <robowflex_library/benchmarking.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    startROS(argc, argv);

    // Create a WAM7 robot, specifying all necessary files.
    auto wam7 = std::make_shared<Robot>("wam7");
    wam7->initialize("package://barrett_model/robots/wam7_bhand.urdf.xacro",          // urdf
                     "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                     "package://barrett_wam_moveit_config/config/joint_limits.yaml",  // joint limits
                     "package://barrett_wam_moveit_config/config/kinematics.yaml"     // kinematics
    );

    // Load kinematics for the WAM7 arm.
    wam7->loadKinematics("arm");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(wam7);

    // Create the default OMPL planner, with the WAM7 planning configuration.
    robow_tesseract::Settings settings;
    settings.simplify_solutions = true;
    settings.use_continuous_validator = true;
    auto planner = std::make_shared<robow_tesseract::OMPLChainPlanner>(wam7);
    planner->initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml",  // planner config
                        settings
    );

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, "arm"));
    request->setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request->setGoalConfiguration({0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    // Setup a benchmarking request for the motion plan requests.
    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("test", scene, planner, request);

    // Set benchmarking options to only compute and store paths and path length.
    Benchmarker::Options options(10, Benchmarker::MetricOptions::PATH | Benchmarker::MetricOptions::LENGTH);

    // Benchmark and output to JSON and a rosbag of trajectories.
    benchmark.benchmark({std::make_shared<JSONBenchmarkOutputter>("test_log.json"),  //
                         std::make_shared<TrajectoryBenchmarkOutputter>("test_log.bag")},
                        options);

    return 0;
}
