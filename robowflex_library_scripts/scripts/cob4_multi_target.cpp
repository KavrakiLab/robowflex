/* Author: Juan D. Hernandez */

// Robowflex
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/detail/cob4.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file cob4_multi_target.cpp
 * A script that demonstrates the multi-target IK query functionality and planning with a cob4 robot. The
 * robowflex_resouces package needs to be available, see https://github.com/KavrakiLab/robowflex_resources.
 * You should run RViz and have a RobotState visualization display enabled set to look at
 * /robowflex/robot_description, and robowflex/state. Also a MarkerArray should be added to visualize the
 * target IK poses.
 */

static const std::string LEFT_ARM = "arm_left";
static const std::string RIGHT_ARM = "arm_right";
static const std::string BOTH_ARMS = "both_arms";
static const std::string LEFT_EE = "arm_left_7_link";
static const std::string RIGHT_EE = "arm_right_7_link";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Care-O-Bot4 robot.
    auto cob4 = std::make_shared<Cob4Robot>();
    cob4->initialize();

    // Load kinematics for the COB4 robot.
    cob4->loadKinematics(BOTH_ARMS);

    // Fold the arms.
    cob4->setGroupState(BOTH_ARMS, {-1.14, -1.50, 0.34, -1.50, 0.43, -1.56, -1.20, 2.69, 1.70, -0.91, 1.50,
                                    -2.14, -2.35, 1.06});

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(cob4);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_cob4.yml");

    // Visualize the Baxter robot
    IO::RVIZHelper rviz(cob4);
    rviz.visualizeCurrentState();
    rviz.updateScene(scene);
    RBX_INFO("Care-O-Bot4 with both_arms folded is visualized. Press enter to continue...");
    std::cin.ignore();

    // Create the default planner for the COB4.
    auto planner = std::make_shared<OMPL::Cob4OMPLPipelinePlanner>(cob4);
    planner->initialize();

    // Create a motion planning request with a joint position goal for the right arm.
    MotionRequestBuilder request_both_arms(planner, BOTH_ARMS);
    request_both_arms.setStartConfiguration(cob4->getScratchState());

    // Create a motion planning request with a pose goal for both_arms
    const auto &goal_pose_right = TF::createPoseQ(0.6, -0.26, 0.97, 1.0, 0.0, 0.0, 0.0);
    const auto &goal_pose_left = TF::createPoseQ(0.38, 0.26, 0.78, 0.0, 0.707, 0.0, 0.707);

    // Visualizing the target poses.
    rviz.addTransformMarker("pose_right", "base_link", goal_pose_right);
    rviz.addTransformMarker("pose_left", "base_link", goal_pose_left);
    rviz.updateMarkers();
    RBX_INFO("Target IK poses are visualized. Press enter to continue...");
    std::cin.ignore();

    Robot::IKQuery query(BOTH_ARMS, {goal_pose_left, goal_pose_right}, {LEFT_EE, RIGHT_EE});
    if (not cob4->setFromIK(query))
    {
        RBX_ERROR("IK query failed!");
        return 1;
    }

    // Visualize resulting state.
    rviz.visualizeCurrentState();
    request_both_arms.setGoalConfiguration(cob4->getScratchState());
    RBX_INFO("Solution to IK is visualized and set as goal for both_arms! Press enter to plan...");
    std::cin.ignore();

    // Do motion planning!
    request_both_arms.setConfig("RRTConnect");
    planning_interface::MotionPlanResponse res = planner->plan(scene, request_both_arms.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    // Visualize resulting state.
    auto both_arms_trajectory = std::make_shared<Trajectory>(res.trajectory_);
    cob4->setGroupState(BOTH_ARMS, both_arms_trajectory->vectorize().back());

    rviz.visualizeCurrentState();

    RBX_INFO("Trajectory visualize. Press enter to exit.");
    std::cin.get();

    return 0;
}
