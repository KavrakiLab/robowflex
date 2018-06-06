#include <ros/ros.h>
#include <signal.h>

#include "robowflex.h"

void shutdown(int sig)
{
    ros::spinOnce();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);

    ros::NodeHandle nh("~");

    robowflex::RobotDescription("ur5",                                                                 // name
                                "package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro",  // urdf
                                "package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf",     // srdf
                                "package://ur5_robotiq85_moveit_config/config/joint_limits.yaml",      // joint limits
                                "package://ur5_robotiq85_moveit_config/config/kinematics.yaml"         // kinematics
    );

    robowflex::RobotModel("ur5");

    ros::spin();
    return 0;
}
