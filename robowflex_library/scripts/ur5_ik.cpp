/* Author: Zachary Kingston */

#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file ur5_ik.cpp
 * Simple demonstration of how to use IK.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();
    ur5->setGroupState("manipulator", {0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    // Use IK to shift robot arm over by desired amount.
    RobotPose goal_pose = ur5->getLinkTF("ee_link");
    goal_pose.translate(Eigen::Vector3d{0.0, -0.3, 0.0});

    if (not ur5->setFromIK(Robot::IKQuery("manipulator", goal_pose)))
    {
        RBX_ERROR("IK Failed");
        return 1;
    }

    return 0;
}
