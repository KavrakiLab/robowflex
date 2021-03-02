#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file wam7_benchmark.cpp
 * A basic script that demonstrates benchmarking with the WAM7 arm. Here, a
 * scene is loaded from an OpenRAVE scene file. Benchmarking output is saved in
 * a JSON file and a ROS bag of trajectories.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

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
    scene->fromOpenRAVEXMLFile("package://optplanners_openrave/scripts/data/envs/wam7_realistic.env.xml");

    // Create the default OMPL planner, with the WAM7 planning configuration.
    auto planner = std::make_shared<OMPL::OMPLPipelinePlanner>(wam7);
    planner->initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, "arm"));
    request->setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request->setGoalConfiguration({0.0, 1.89, 0.0, -0.3, 1.3, 0.0, 0.2});

    // Setup a benchmarking request for the motion plan requests.
    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("test", scene, planner, request);

    // Set benchmarking options to only compute and store paths and path length.
    Benchmarker::Options options(10,                                                                     //
                                 Benchmarker::MetricOptions::PATH | Benchmarker::MetricOptions::LENGTH,  //
                                 0.1);

    // Benchmark and output to JSON and a rosbag of trajectories.
    benchmark.benchmark({std::make_shared<JSONBenchmarkOutputter>("test_log.json"),  //
                         std::make_shared<TrajectoryBenchmarkOutputter>("test_log.bag")},
                        options);

    return 0;
}
