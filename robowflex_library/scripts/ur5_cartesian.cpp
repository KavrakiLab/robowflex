/* Author: Zachary Kingston */

#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

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

    // Create a Cartesian planner for the UR5.
    auto cartesian_planner = std::make_shared<SimpleCartesianPlanner>(ur5, "cartesian");

    EigenSTL::vector_Vector3d directions = {
        {0.0, -0.3, 0.0}, {0.0, 0.0, -0.2}, {0.0, 0.3, 0.0},  {0.0, 0.0, 0.2},  // Move in an YZ rectangle
        {0.2, 0.0, 0.0},  {0.0, -0.3, 0.0}, {-0.2, 0.0, 0.0}, {0.0, 0.3, 0.0},  // Move in an XY rectangle
        {0.4, -0.3, 0.2}, {-0.4, 0.3, -0.2}                                     // Diagonal move
    };

    for (const auto &direction : directions)
    {
        RBX_INFO("Moving end-effector in direction [%1%, %2%, %3%]",  //
                 direction[0], direction[1], direction[2]);

        // Visualize the scene.
        scene->getCurrentState() = *ur5->getScratchState();
        rviz.updateScene(scene);

        // Create the IK Query, and then set the scene for collision checking.
        // Uses directional offset IKQuery constructor.
        Robot::IKQuery query("manipulator", "ee_link", *ur5->getScratchState(), direction);
        query.scene = scene;

        // Plan for a straight interpolation of the end-effector to the query.
        auto response = cartesian_planner->plan(*ur5->getScratchState(), query);
        if (response.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            RBX_ERROR("Planning failed!");
            return 1;
        }

        // Publish the trajectory to a topic to display in RViz
        Trajectory trajectory(response.trajectory_);
        rviz.updateTrajectory(trajectory);

        // Set the scratch state to the end of the computed trajectory.
        ur5->setState(trajectory.getFinalPositions());

        RBX_INFO("Press enter to continue to next direction.");
        std::cin.get();
    }

    // Visualize the final pose.
    scene->getCurrentState() = *ur5->getScratchState();
    rviz.updateScene(scene);

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
