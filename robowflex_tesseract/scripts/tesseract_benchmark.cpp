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

    // Create a UR5 robot, specifying all necessary files.
    auto ur5_plain = std::make_shared<robowflex::Robot>("ur5");
    ur5_plain->initialize("package://ur_description/urdf/ur5_joint_limited_robot.urdf.xacro",  // urdf
                          "package://ur5_moveit_config/config/ur5.srdf",                       // srdf
                          "package://ur5_moveit_config/config/joint_limits.yaml",              // joint limits
                          "package://ur5_moveit_config/config/kinematics.yaml"                 // kinematics
    );

    // Load kinematics for the UR5 arm.
    ur5_plain->loadKinematics("manipulator");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5_plain);

    // Create the default OMPL planner, with the WAM7 planning configuration.
    hypercube::Settings settings;
    settings.simplify_solutions = true;
    settings.use_continuous_validator = true;
    auto planner_continuous = std::make_shared<hypercube::OMPLChainPlanner>(ur5_plain);
    planner_continuous->initialize("package://ur5_moveit_config/config/ompl_planning.yaml", settings);

    hypercube::Settings discrete_settings;
    discrete_settings.simplify_solutions = true;
    settings.use_continuous_validator = false;
    auto planner_discrete = std::make_shared<hypercube::OMPLChainPlanner>(ur5_plain);
    planner_discrete->initialize("package://ur5_moveit_config/config/ompl_planning.yaml", discrete_settings);

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner_continuous, "manipulator"));
    request->setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request->setGoalConfiguration({0.0, -1.0, 0.0, 0.0, 0.0, 0.0});

    // Setup a benchmarking request for the motion plan requests.
    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("continuous", scene, planner_continuous, request);

    // Set benchmarking options to only compute and store paths and path length.
    Benchmarker::Options options(50, Benchmarker::MetricOptions::PATH | Benchmarker::MetricOptions::LENGTH |
                                         Benchmarker::MetricOptions::CORRECT);

    // Benchmark and output to JSON and a rosbag of trajectories.
    benchmark.benchmark({std::make_shared<JSONBenchmarkOutputter>("test_log.json"),  //
                         std::make_shared<TrajectoryBenchmarkOutputter>("test_log.bag")},
                        options);

    return 0;
}
