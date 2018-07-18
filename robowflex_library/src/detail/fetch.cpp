/* Author: Zachary Kingston */

#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

const std::string FetchRobot::URDF{"package://fetch_description/robots/fetch.urdf"};
const std::string FetchRobot::SRDF{"package://fetch_moveit_config/config/fetch.srdf"};
const std::string FetchRobot::LIMITS{"package://fetch_moveit_config/config/joint_limits.yaml"};
const std::string FetchRobot::KINEMATICS{"package://fetch_moveit_config/config/kinematics.yaml"};

const std::string  //
    OMPL::FetchOMPLPipelinePlanner::CONFIG{"package://fetch_moveit_config/config/ompl_planning.yaml"};

FetchRobot::FetchRobot() : Robot("ur5")
{
}

bool FetchRobot::initialize()
{
    bool success = Robot::initialize(URDF, SRDF, LIMITS, KINEMATICS);

    loadKinematics("arm");
    loadKinematics("arm_with_torso");

    return success;
}

OMPL::FetchOMPLPipelinePlanner::FetchOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::FetchOMPLPipelinePlanner::initialize(const Settings &settings, const std::string &config_file,
                                                const std::string &plugin,
                                                const std::vector<std::string> &adapters)
{
    return OMPLPipelinePlanner::initialize(config_file, settings, plugin, adapters);
}
