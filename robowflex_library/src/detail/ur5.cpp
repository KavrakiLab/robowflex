/* Author: Zachary Kingston */

#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

const std::string UR5Robot::URDF{"package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro"};
const std::string UR5Robot::SRDF{"package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf"};
const std::string UR5Robot::LIMITS{"package://ur5_robotiq85_moveit_config/config/joint_limits.yaml"};
const std::string UR5Robot::KINEMATICS{"package://ur5_robotiq85_moveit_config/config/kinematics.yaml"};

const std::string  //
    OMPL::UR5OMPLPipelinePlanner::CONFIG{"package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"};

UR5Robot::UR5Robot() : Robot("ur5")
{
}

bool UR5Robot::initialize()
{
    bool success = Robot::initialize(URDF, SRDF, LIMITS, KINEMATICS);
    loadKinematics("manipulator");

    return success;
}

OMPL::UR5OMPLPipelinePlanner::UR5OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::UR5OMPLPipelinePlanner::initialize(const Settings settings, const std::string &config_file,
                                              const std::string &plugin,
                                              const std::vector<std::string> &adapters)
{
    return OMPLPipelinePlanner::initialize(config_file, settings, plugin, adapters);
}
