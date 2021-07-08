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

// const std::vector<std::string> tips = {"ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "wrist"};
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

    shadowhand->setState({"kuka_arm_1_joint", "kuka_arm_3_joint"}, {0.3, 0.3});

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

    Robot::IKQuery query("all_fingers", poses, tips);
    query.options.return_approximate_solution = true;
    // query.scene = scene;
    // query.verbose = true;

    if (not shadowhand->setFromIK(query))
    {
        RBX_ERROR("IK query failed!");
    }

    auto result = scene->checkCollision(*shadowhand->getScratchState());
    result.print();

    // Visualize resulting state.
    rviz.visualizeCurrentState();
    RBX_INFO("Solution to IK is visualized. Press enter to exit...");
    std::cin.ignore();

    // ros::NodeHandle handle("/");
    // auto sub = handle.subscribe<sensor_msgs::JointState>("/robowflex/joint_states", 1,
    //                                                      [&](const sensor_msgs::JointState::ConstPtr &msg)
    //                                                      {
    //                                                          RBX_INFO("Message Recieved!");
    //                                                          shadowhand->setState(*msg);
    //                                                          rviz.visualizeCurrentState();
    //                                                      });
    // ros.wait();

    rviz.removeAllMarkers();
    rviz.updateMarkers();

    return 0;
}
