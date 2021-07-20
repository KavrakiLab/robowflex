/* Author: Zachary Kingston */

#include <sensor_msgs/JointState.h>

// Robowflex
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file shadowhand_ik.cpp
 * Example of multi-target IK using BioIK for the shadowhand. This script also uses some of the more advanced
 * features of Robot::IKQuery and setFromIK() that allow for approximate solutions and higher thresholds on IK
 * solution tolerance.
 * This script also visualizes the query and result using RViz. See
 * https://kavrakilab.github.io/robowflex/rviz.html for RViz visualization.
 */

// Tip links of each of the shadowhand's fingers (from forefinger to thumb).
const std::vector<std::string> tips = {"fftip", "mftip", "rftip", "lftip", "thtip"};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create a Shadowhand robot and initialize.
    auto shadowhand = std::make_shared<Robot>("shadowhand");
    shadowhand->initializeFromYAML("package://robowflex_resources/shadowhand.yml");
    shadowhand->loadKinematics("all_fingers", false);
    shadowhand->setState({"kuka_arm_1_joint", "kuka_arm_3_joint", "kuka_arm_5_joint"}, {1, 2, 1});

    auto scene = std::make_shared<Scene>(shadowhand);
    scene->updateCollisionObject("rod_1", Geometry::makeCylinder(0.1, 1),
                                 TF::createPoseXYZ(0, 0, 0.5, constants::pi / 2, 0, 0));
    scene->updateCollisionObject("rod_2", Geometry::makeCylinder(0.1, 1),
                                 TF::createPoseXYZ(-0.2, 0.2, 0.5, 0, 0, 0));
    scene->updateCollisionObject("rod_3", Geometry::makeCylinder(0.01, 0.25),
                                 TF::createPoseXYZ(0, -0.04, 0.95, 0, constants::pi / 2, 0));

    // Save initial state.
    auto start = *shadowhand->getScratchState();

    // Visualize the Shadowhand robot.
    IO::RVIZHelper rviz(shadowhand);
    rviz.updateScene(scene);

    // Set the shadowhand to a default grasp and save that state.
    shadowhand->setStateFromYAMLFile("package://robowflex_resources/shadowhand/poses/grasp.yml");
    auto grasp = *shadowhand->getScratchState();

    // Visualize desired grasp state.
    rviz.visualizeCurrentState();
    RBX_INFO("Desired Grasp Displayed, Press Enter to Continue...");
    std::cin.ignore();

    // Extract poses of the tip links.
    RobotPoseVector poses;
    for (const auto &tip : tips)
    {
        auto pose = shadowhand->getLinkTF(tip);
        poses.emplace_back(pose);

        // Create a marker at each tip frame.
        rviz.addTransformMarker(tip + "_goal", "map", pose, 0.3);
    }

    // Reset state to the initial start state.
    *shadowhand->getScratchState() = start;

    // Visualize starting state and goal markers.
    rviz.visualizeCurrentState();
    rviz.updateMarkers();
    RBX_INFO("Initial State and Goal Displayed, Press Enter to Continue...");
    std::cin.ignore();

    // Configure query for shadowhand
    Robot::IKQuery query("all_fingers", poses, tips);
    query.scene = scene;
    query.verbose = true;  // If verbose is true, debugging output for constraints and collisions is output.

    query.timeout = 0.05;
    query.attempts = 50;

    // Need approximate solutions as multi-target BioIK will only return approximate solutions.
    query.options.return_approximate_solution = true;
    query.validate = true;  // Need external validation to verify approximate solutions are within tolerance.
    query.valid_distance = 0.01;  // Tuned distance threshold that is appropriate for query.

    // By adding metrics to the IKQuery, every attempt is used to find the best configuration (according to
    // the sum of the metrics). A weighting factor is added to balance the metrics.
    query.addDistanceMetric();       // Distance to IKQuery
    query.addCenteringMetric(0.01);  // Distance of joints from their center
    query.addClearanceMetric(0.01);  // Distance of robot to collision

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
