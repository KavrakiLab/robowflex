/* Author: Juan D. Hernandez */

#include <robowflex_library/log.h>
#include <robowflex_library/detail/cob4.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file cob4_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * COB4 robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, the pose goal, and motion plan
 * displayed in RViz.
 */

static const std::string LEFT_ARM = "arm_left";
static const std::string RIGHT_ARM = "arm_right";
static const std::string LEFT_EE = "arm_left_7_link";
static const std::string RIGHT_EE = "arm_right_7_link";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Care-O-Bot4 robot.
    auto cob4 = std::make_shared<Cob4Robot>();
    cob4->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(cob4);
    IO::RobotBroadcaster bc(cob4);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load kinematics for the COB4 robot.
    cob4->loadKinematics(RIGHT_ARM);
    cob4->loadKinematics(LEFT_ARM);

    // Fold the arms.
    cob4->setGroupState(RIGHT_ARM, {2.69, 1.70, -0.91, 1.50, -2.14, -2.35, 1.06});
    cob4->setGroupState(LEFT_ARM, {-1.14, -1.50, 0.34, -1.50, 0.43, -1.56, -1.20});

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(cob4);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_cob4.yml");

    // Visualize the scene.
    rviz.visualizeCurrentState();
    rviz.updateScene(scene);

    // Create the default planner for the COB4.
    auto planner = std::make_shared<OMPL::Cob4OMPLPipelinePlanner>(cob4);
    planner->initialize();

    // Sets the Cob4's base pose.
    cob4->pointHead({0.6, -0.26, 0.95});

    // Create a motion planning request with a joint position goal for the right arm.
    MotionRequestBuilder request_right_arm(planner, RIGHT_ARM);
    request_right_arm.setStartConfiguration(cob4->getScratchState());

    // Create a motion planning request with a pose goal for the right arm.
    const auto &goal_pose_right = TF::createPoseQ(0.6, -0.26, 0.95, 1.0, 0.0, 0.0, 0.0);
    request_right_arm.setGoalPose(RIGHT_EE, "base_link", goal_pose_right);

    // Visualizing the target poses for the right arm.
    rviz.addGoalMarker("goal", request_right_arm);
    rviz.addTransformMarker("pose_right", "base_link", goal_pose_right);
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed for the right arm! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    request_right_arm.setConfig("RRTConnect");
    planning_interface::MotionPlanResponse res = planner->plan(scene, request_right_arm.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    // Visualize resulting state.
    // Create a trajectory object for better manipulation.
    auto right_arm_trajectory = std::make_shared<Trajectory>(res.trajectory_);
    cob4->setGroupState(RIGHT_ARM, right_arm_trajectory->vectorize().back());

    rviz.visualizeCurrentState();

    RBX_INFO("Press enter to continue with the left arm.");

    // Sets the Cob4's base pose.
    cob4->pointHead({0.4, 0.26, 0.79});

    // Create a motion planning request with a joint position goal for the right arm.
    MotionRequestBuilder request_left_arm(planner, LEFT_ARM);
    request_left_arm.setStartConfiguration(cob4->getScratchState());

    // Create a motion planning request with a pose goal for the left arm.
    const auto &goal_pose_left = TF::createPoseQ(0.38, 0.26, 0.76, 0.0, 0.707, 0.0, 0.707);
    request_left_arm.setGoalPose(LEFT_EE, "base_link", goal_pose_left);

    // Visualizing the target poses for the left arm.
    rviz.addGoalMarker("goal_left", request_left_arm);
    rviz.addTransformMarker("pose_left", "base_link", goal_pose_left);
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed for the left arm! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    request_left_arm.setConfig("RRTConnect");
    res = planner->plan(scene, request_left_arm.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    // Visualize resulting state.
    auto left_arm_trajectory = std::make_shared<Trajectory>(res.trajectory_);
    cob4->setGroupState(LEFT_ARM, left_arm_trajectory->vectorize().back());

    rviz.visualizeCurrentState();

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
