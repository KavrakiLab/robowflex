/* Author: Zachary Kingston */

#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string  //
    UR5Robot::DEFAULT_URDF{"package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro"};
const std::string  //
    UR5Robot::DEFAULT_SRDF{"package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf"};
const std::string  //
    UR5Robot::DEFAULT_LIMITS{"package://ur5_robotiq85_moveit_config/config/joint_limits.yaml"};
const std::string  //
    UR5Robot::DEFAULT_KINEMATICS{"package://ur5_robotiq85_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::UR5OMPLPipelinePlanner::DEFAULT_CONFIG{
        "package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  //
    };

const std::string  //
    UR5Robot::RESOURCE_URDF{"package://robowflex_resources/ur/robots/ur5_robotiq_robot_limited.urdf.xacro"};
const std::string  //
    UR5Robot::RESOURCE_SRDF{"package://robowflex_resources/ur/config/ur5/ur5_robotiq85.srdf.xacro"};
const std::string  //
    UR5Robot::RESOURCE_LIMITS{"package://robowflex_resources/ur/config/ur5/joint_limits.yaml"};
const std::string  //
    UR5Robot::RESOURCE_KINEMATICS{"package://robowflex_resources/ur/config/ur5/kinematics.yaml"};
const std::string  //
    OMPL::UR5OMPLPipelinePlanner::RESOURCE_CONFIG{
        "package://robowflex_resources/ur/config/ur5/ompl_planning.yaml"  //
    };

UR5Robot::UR5Robot() : Robot("ur5")
{
}

bool UR5Robot::initialize()
{
    bool success = false;

    // First attempt the `robowflex_resources` package, then attempt the "actual" resource files.
    if (IO::resolvePackage(RESOURCE_URDF).empty() or IO::resolvePackage(RESOURCE_SRDF).empty())
    {
        RBX_INFO("Initializing UR5 with `ur_description`");
        success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);
    }
    else
    {
        RBX_INFO("Initializing UR5 with `robowflex_resources`");
        success = Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS, RESOURCE_KINEMATICS);
    }

    loadKinematics("manipulator");

    return success;
}

OMPL::UR5OMPLPipelinePlanner::UR5OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::UR5OMPLPipelinePlanner::initialize(const Settings &settings,
                                              const std::vector<std::string> &adapters)
{
    if (IO::resolvePackage(RESOURCE_CONFIG).empty())
        return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
    return OMPLPipelinePlanner::initialize(RESOURCE_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}
