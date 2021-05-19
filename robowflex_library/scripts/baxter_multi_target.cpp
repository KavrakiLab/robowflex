/* Author: Constantinos Chamzas */

// Robowflex
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>

using namespace robowflex;

/* \file baxter_multi_target.cpp
 * A script that demonstrates the multi-target IK query functionality with a baxter robot.  The
 * robowflex_resouces package needs to  be available, see https://github.com/KavrakiLab/robowflex_resources.
 * You should run RViz and have a RobotState visualization display enabled set to look at
 * /robowflex/robot_description, and robowflex/state. Also a MarkerArray should be added to visualize the
 * target IK poses.
 */

static const std::string BOTH_ARMS = "both_arms";
static const std::string LEFT_ARM = "left_arm";
static const std::string RIGHT_ARM = "right_arm";
static const std::string LEFT_EE = "left_gripper_base";
static const std::string RIGHT_EE = "right_gripper_base";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto baxter = std::make_shared<Robot>("baxter");

    // Initialize baxter from robowflex_resources.
    baxter->initializeFromYAML("package://robowflex_resources/baxter.yml");
    baxter->loadKinematics(LEFT_ARM);
    baxter->loadKinematics(RIGHT_ARM);

    IO::RVIZHelper rviz(baxter);
    rviz.visualizeCurrentState();
    RBX_INFO("Default robot state is visualized. \n Press enter to continue...");
    std::cin.ignore();

    // an EE offset that makes the arm point upwards.
    auto ee_offset = TF::createPoseXYZ(0, 0, -1.3, constants::pi, 0, 0);
    // Use IK to shift robot arm over by desired amount.
    RobotPose goal_pose_left = baxter->getLinkTF(LEFT_EE) * ee_offset;
    RobotPose goal_pose_right = baxter->getLinkTF(RIGHT_EE) * ee_offset;

    // Visualizing the target poses.
    rviz.addTransformMarker("pose_right", "map", goal_pose_right);
    rviz.addTransformMarker("pose_left", "map", goal_pose_left);
    rviz.updateMarkers();
    RBX_INFO("Target IK poses are visualized. \n Press enter to continue...");
    std::cin.ignore();

    const auto ik_query = Robot::IKQuery(BOTH_ARMS, {goal_pose_left, goal_pose_right}, {LEFT_EE, RIGHT_EE});

    if (not baxter->setFromIK(ik_query))
    {
        RBX_INFO("IK query failed!");
        exit(-1);
    }
    rviz.visualizeCurrentState();
    RBX_INFO("Solution to IK is visualized. Press enter to exit...");
    std::cin.ignore();

    exit(0);
}
