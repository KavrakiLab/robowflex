#include <ros/ros.h>
#include <signal.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>

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

    robowflex::loadRobotDescription("ur5",                                                                 // name
                                    "package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro",  // urdf
                                    "package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf",     // srdf
                                    "package://ur5_robotiq85_moveit_config/config/joint_limits.yaml",  // joint limits
                                    "package://ur5_robotiq85_moveit_config/config/kinematics.yaml"     // kinematics
    );

    robot_model::RobotModelPtr model_(robowflex::loadRobotModel("ur5"));
    planning_scene::PlanningScene ps(model_);

    // planning_interface::PlannerManager pm();
    // pm.initialize();

    // auto pc = pm.getPlanningContext();

    return 0;
}
