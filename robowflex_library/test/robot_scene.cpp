/* Author: Zachary Kingston, Carlos Quintero Pena */

#include <gtest/gtest.h>

#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>

using namespace robowflex;

namespace
{
    UR5RobotPtr getUR5Robot()
    {
        static UR5RobotPtr ur5;

        // Create the default UR5 robot.
        if (not ur5)
        {
            ur5 = std::make_shared<UR5Robot>();
            ur5->initialize();
        }

        return ur5;
    }
}  // namespace

TEST(Scene, detatchObject)
{
    auto ur5 = getUR5Robot();

    // Create an empty scene and add a cylinder to it
    auto scene = std::make_shared<Scene>(ur5);

    const auto cylinder_position = Eigen::Vector3d{-0.270, 0.42, 1.1572};  // Initial cylinder position.
    const auto shift = Eigen::Vector3d{0.0, -0.3, 0.0};   // Desired offset movement of cylinder.
    const auto desired_pose = cylinder_position + shift;  // Desired final position of cylinder.

    // Add cylinder to scene.
    scene->updateCollisionObject(
        "cylinder",                          //
        Geometry::makeCylinder(0.025, 0.1),  //
        TF::createPoseQ(cylinder_position, Eigen::Quaterniond{0.707, 0.0, 0.707, 0.0}));

    // Set start configuration to the robot scratch state and attach the cylinder to it
    ur5->setGroupState("manipulator", {0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});
    scene->attachObject(*ur5->getScratchState(), "cylinder");

    // Use IK to shift robot arm over by desired amount.
    RobotPose goal_pose = ur5->getLinkTF("ee_link");
    goal_pose.translate(shift);

    bool success = ur5->setFromIK(Robot::IKQuery("manipulator", goal_pose));
    ASSERT_TRUE(success);

    // Get new position for the cylinder and compare to its theoretical value.
    scene->detachObject(*ur5->getScratchState(), "cylinder");
    auto new_pose = scene->getObjectPose("cylinder");
    auto diff = desired_pose - new_pose.translation();

    ASSERT_NEAR(diff.norm(), 0., 0.005);
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
