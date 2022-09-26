/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file ur5_cylinder.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the UR5
 * robot. Here, the goal is created using the `addCylinderSideGrasp` function,
 * which creates a more complicated grasp to grasp any side of a cylinder. See
 * https://kavrakilab.github.io/robowflex/rviz.html for how to use RViz
 * visualization. Here, the scene, the pose goal, and motion plan displayed in
 * RViz.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(ur5);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Create the cylinder we want to grasp
    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});

    auto cylinder = Geometry::makeCylinder(0.025, 0.1);

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);
    scene->updateCollisionObject("cylinder", cylinder, pose);

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    request.addCylinderSideGrasp("ee_link", "world",  //
                                 pose, cylinder,      //
                                 0.15, 0.04, 16);     //

    rviz.addGoalMarker("goal", request);  // Visualize the grasping regions
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    rviz.removeMarker("goal");
    rviz.updateMarkers();

    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
