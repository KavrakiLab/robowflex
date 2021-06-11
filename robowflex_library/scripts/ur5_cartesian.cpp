/* Author: Zachary Kingston */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/util.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/log.h>

using namespace robowflex;

/* \file ur5_cartesian.cpp
 * Simple demonstration of how to use the SimpleCartesianPlanner with the UR5 combined with visualization.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();
    ur5->setGroupState("manipulator", {0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(ur5);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    auto scene = std::make_shared<Scene>(ur5);
    scene->getCurrentState() = *ur5->getScratchState();

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create a Cartesian planner for the UR5.
    auto cartesian_planner = std::make_shared<SimpleCartesianPlanner>(ur5, "cartesian");

    // Use IK to shift robot arm over by desired amount.
    RobotPose goal_pose = ur5->getLinkTF("ee_link");
    goal_pose.translate(Eigen::Vector3d{0.0, -0.3, 0.0});

    // Create the IK Query, and then set the scene for collision checking.
    Robot::IKQuery query("manipulator", goal_pose);
    query.scene = scene;

    // Plan for a straight interpolation of the end-effector to the query.
    auto response = cartesian_planner->plan(*ur5->getScratchState(), query);
    if (response.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        RBX_ERROR("Planning failed!");
        return 1;
    }

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(response);

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
