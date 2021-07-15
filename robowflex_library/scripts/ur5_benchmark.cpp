/* Author: Zachary Kingston */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file ur5_benchmark.cpp
 * A basic script that demonstrates benchmarking with the UR5 robot.
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilderPtr joint_request(new MotionRequestBuilder(planner, "manipulator"));
    joint_request->setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});
    joint_request->setGoalConfiguration({-0.39, -0.69, -2.12, 2.82, -0.39, 0});

    // Create a motion planning request with a pose goal.
    MotionRequestBuilderPtr pose_request(new MotionRequestBuilder(planner, "manipulator"));
    pose_request->setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    pose_request->setGoalRegion("ee_link", "world",               // links
                                pose, Geometry::makeSphere(0.1),  // position
                                orn, {0.01, 0.01, 0.01}           // orientation
    );

    Profiler::Options options;
    Experiment experiment("ur5_demo",  // Name of experiment
                          options,     // Options for internal profiler
                          5.0,         // Timeout allowed for ALL queries
                          50);         // Number of trials

    experiment.addQuery("joint", scene, planner, joint_request);
    experiment.addQuery("pose", scene, planner, pose_request);

    auto dataset = experiment.benchmark(4);

    OMPLPlanDataSetOutputter output("robowflex_ur5_demo");
    output.dump(*dataset);

    return 0;
}
