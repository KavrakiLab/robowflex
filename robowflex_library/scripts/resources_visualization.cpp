/* Author: Zachary Kingston */

#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/** \file
 *  A helper script to visualize all the robots available in robowflex_resources, see
 * https://github.com/KavrakiLab/robowflex_resources. You should run RViz and have a PlanningScene
 * visualization display enabled set to look at /robowflex/robot_description. Enable/Disable the PlanningScene
 * display after running the script with a robot to see it displayed.
 *
 * Currently, the robots available are the Fetch, UR5, Panda, Baxter, and YuMi. See output for details.
 */

// Fetch robot
const std::string FETCH[4] = {
    "package://robowflex_resources/fetch/robots/fetch.urdf",         // urdf
    "package://robowflex_resources/fetch/config/fetch.srdf",         // srdf
    "package://robowflex_resources/fetch/config/joint_limits.yaml",  // joint limits
    "package://robowflex_resources/fetch/config/kinematics.yaml"     // kinematics
};

// UR5 with Robotiq 85 Gripper on a table
const std::string UR5[4] = {
    "package://robowflex_resources/ur/robots/ur5_robotiq_robot_limited.urdf.xacro",  // urdf
    "package://robowflex_resources/ur/config/ur5/ur5_robotiq85_table.srdf.xacro",    // srdf
    "package://robowflex_resources/ur/config/ur5/joint_limits.yaml",                 // joint limits
    "package://robowflex_resources/ur/config/ur5/kinematics.yaml"                    // kinematics
};

// Franka Emika Panda robot with hand
const std::string PANDA[4] = {
    "package://robowflex_resources/panda/urdf/panda.urdf",           // urdf
    "package://robowflex_resources/panda/config/panda.srdf",         // srdf
    "package://robowflex_resources/panda/config/joint_limits.yaml",  // joint limits
    "package://robowflex_resources/panda/config/kinematics.yaml"     // kinematics
};

// Rethink Robotics Baxter
const std::string BAXTER[4] = {
    "package://robowflex_resources/baxter/urdf/baxter.urdf.xacro",    // urdf
    "package://robowflex_resources/baxter/config/baxter.srdf",        // srdf
    "package://robowflex_resources/baxter/config/joint_limits.yaml",  // joint limits
    "package://robowflex_resources/baxter/config/kinematics.yaml"     // kinematics
};

// ABB YuMi (IRB 14000)
const std::string YUMI[4] = {
    "package://robowflex_resources/yumi/urdf/yumi.urdf",            // urdf
    "package://robowflex_resources/yumi/config/yumi.srdf",          // srdf
    "package://robowflex_resources/yumi/config/joint_limits.yaml",  // joint limits
    "package://robowflex_resources/yumi/config/kinematics.yaml"     // kinematics
};

// KUKA Robot with mounted Shadowhand Gripper
const std::string SHADOWHAND[4] = {
    "package://robowflex_resources/shadowhand/urdf/shadowhand.urdf",      // urdf
    "package://robowflex_resources/shadowhand/config/shadowhand.srdf",    // srdf
    "package://robowflex_resources/shadowhand/config/joint_limits.yaml",  // joint limits
    "package://robowflex_resources/shadowhand/config/kinematics.yaml"     // kinematics
};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    if (argc != 2)
        RBX_FATAL("Specify robot to load as an argument "
                  "(e.g., `rosrun robowflex_library resources_visualization <robot>`). "
                  "Can be {baxter, ur5, panda, fetch, yumi, shadowhand}.");

    const auto name = std::string(argv[1]);

    // Create an empty robot.
    const auto &robot = std::make_shared<Robot>(name);

    // Initialize robot from argument.
    if (name == "fetch")
        robot->initialize(FETCH[0], FETCH[1], FETCH[2], FETCH[3]);
    else if (name == "ur5")
        robot->initialize(UR5[0], UR5[1], UR5[2], UR5[3]);
    else if (name == "panda")
        robot->initialize(PANDA[0], PANDA[1], PANDA[2], PANDA[3]);
    else if (name == "baxter")
        robot->initialize(BAXTER[0], BAXTER[1], BAXTER[2], BAXTER[3]);
    else if (name == "yumi")
        robot->initialize(YUMI[0], YUMI[1], YUMI[2], YUMI[3]);
    else if (name == "shadowhand")
        robot->initialize(SHADOWHAND[0], SHADOWHAND[1], SHADOWHAND[2], SHADOWHAND[3]);
    else
        RBX_FATAL("Unknown robot. Can be {baxter, ur5, panda, fetch, yumi, shadowhand}.");

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(robot);

    RBX_INFO("Press enter to exit.");
    std::cin.ignore();

    return 0;
}
