/* Author: Zachary Kingston */

#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_cartesian.cpp
 * Simple demonstration of how to use the SimpleCartesianPlanner with the Fetch combined with visualization.
 */

static const std::string GROUP = "arm";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();
    fetch->setGroupState(GROUP, {0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(fetch);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    auto scene = std::make_shared<Scene>(fetch);

    // Create a Cartesian planner for the FETCH.
    auto cartesian_planner = std::make_shared<SimpleCartesianPlanner>(fetch, "cartesian");

    EigenSTL::vector_Vector3d directions = {
        {0.3, 0.0, 0.0},                                      // Extend
        {0.0, -0.1, 0.0}, {0.0, 0.2, 0.0}, {0.0, -0.1, 0.0},  // Wiggle up & down
        {-0.3, 0.0, 0.0},                                     // Retract
    };

    for (const auto &direction : directions)
    {
        RBX_INFO("Moving end-effector in direction [%1%, %2%, %3%]",  //
                 direction[0], direction[1], direction[2]);

        // Visualize the scene.
        scene->getCurrentState() = *fetch->getScratchState();
        rviz.updateScene(scene);

        // Create the IK Query, and then set the scene for collision checking.
        // Uses directional offset IKQuery constructor.
        Robot::IKQuery query(GROUP, "wrist_roll_link", *fetch->getScratchState(), direction);
        query.scene = scene;

        // Plan for a straight interpolation of the end-effector to the query.
        auto response = cartesian_planner->plan(*fetch->getScratchState(), query);
        if (response.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            RBX_ERROR("Planning failed!");
            return 1;
        }

        // Publish the trajectory to a topic to display in RViz
        Trajectory trajectory(response.trajectory_);
        rviz.updateTrajectory(trajectory);

        // Set the scratch state to the end of the computed trajectory.
        fetch->setState(trajectory.getFinalPositions());

        RBX_INFO("Press enter to continue to next direction.");
        std::cin.get();
    }

    // Visualize the final pose.
    scene->getCurrentState() = *fetch->getScratchState();
    rviz.updateScene(scene);

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
