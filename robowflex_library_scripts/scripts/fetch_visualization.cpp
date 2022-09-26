/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * Fetch robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, the pose goal, and motion plan
 * displayed in RViz.
 */

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(fetch);
    IO::RobotBroadcaster bc(fetch);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch.yml");

    // Visualize the scene in RViz.
    rviz.updateScene(scene);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request.setStartConfiguration(fetch->getScratchState());

    // Create a motion planning request with a pose goal. Cube3
    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{0.4, 0.6, 0.92});

    Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};

    auto region = Geometry::makeSphere(0.01);

    request.setGoalRegion("wrist_roll_link", "base_link",  // links
                          pose, region,                    // position
                          orn, {0.1, 0.1, 0.1}             // orientation
    );

    // Display the goal region in RViz.
    rviz.addGoalMarker("goal", request);
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz.
    rviz.updateTrajectory(res);

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("fetch_pick.yml");

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    // Clean up RViz.
    rviz.removeMarker("goal");
    rviz.updateMarkers();
    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
