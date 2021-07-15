/* Author: Zachary Kingston */

#include <sensor_msgs/JointState.h>

// Robowflex
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file shadowhand_ik.cpp
 */

// const std::vector<std::string> tips = {"ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal",
// "wrist"};
const std::vector<std::string> tips = {"fftip", "mftip", "rftip", "lftip", "thtip"};
// const std::vector<std::string> tips = {"ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal"};
// const std::vector<std::string> tips = {"ffdistal", "thdistal", "wrist"};
// const std::vector<std::string> tips = {"fftip", "thtip"};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create a Shadowhand robot.
    auto shadowhand = std::make_shared<Robot>("shadowhand");
    shadowhand->initializeFromYAML("package://robowflex_resources/shadowhand.yml");
    shadowhand->loadKinematics("all_fingers", false);

    shadowhand->setState({"kuka_arm_1_joint", "kuka_arm_3_joint"}, {0.6, 0.6});

    auto scene = std::make_shared<Scene>(shadowhand);

    auto start = *shadowhand->getScratchState();

    // Visualize the Shadowhand robot
    IO::RVIZHelper rviz(shadowhand);

    shadowhand->setStateFromYAMLFile("package://robowflex_resources/shadowhand/poses/grasp.yml");
    auto grasp = *shadowhand->getScratchState();

    rviz.visualizeCurrentState();
    RBX_INFO("Desired Grasp Displayed, Press Enter to Continue...");
    std::cin.ignore();

    RobotPoseVector poses;
    for (const auto &tip : tips)
    {
        auto pose = shadowhand->getLinkTF(tip);
        poses.emplace_back(pose);

        rviz.addTransformMarker(tip + "_goal", "map", pose, 0.3);
    }

    // Reset
    *shadowhand->getScratchState() = start;

    rviz.visualizeCurrentState();
    rviz.updateMarkers();
    RBX_INFO("Initial State and Goal Displayed, Press Enter to Continue...");
    std::cin.ignore();

    // Configure query for shadowhand
    Robot::IKQuery query("all_fingers", poses, tips);
    query.timeout = 0.1;
    query.attempts = 10;
    query.options.return_approximate_solution = true;

    query.scene = scene;
    query.validate = true;
    query.valid_distance = 0.01;

    query.addDistanceMetric();
    query.addCenteringMetric();

    if (not shadowhand->setFromIK(query))
        RBX_ERROR("IK query failed!");

    // Visualize resulting state.
    rviz.visualizeCurrentState();
    RBX_INFO("Solution to IK is visualized. Press enter to exit...");
    std::cin.ignore();

    rviz.removeAllMarkers();
    rviz.updateMarkers();

    return 0;
}
