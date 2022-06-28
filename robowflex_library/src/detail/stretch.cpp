/* Author: Carlos Quintero-Peña */

#include <cmath>

#include <robowflex_library/detail/stretch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string StretchRobot::DEFAULT_URDF{"package://stretch_description/urdf/stretch_description.xacro"};
const std::string StretchRobot::DEFAULT_SRDF{"package://stretch_moveit_config/config/"
                                             "stretch_description.srdf"};
const std::string StretchRobot::DEFAULT_LIMITS{"package://stretch_moveit_config/config/joint_limits.yaml"};
const std::string StretchRobot::DEFAULT_KINEMATICS{"package://stretch_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::StretchOMPLPipelinePlanner::DEFAULT_CONFIG{"package://stretch_moveit_config/config/"
                                                     "ompl_planning.yaml"};

const std::string StretchRobot::RESOURCE_URDF{"package://robowflex_resources/stretch/urdf/"
                                              "stretch_description.xacro"};
const std::string StretchRobot::RESOURCE_SRDF{"package://robowflex_resources/stretch/config/"
                                              "stretch_description.srdf"};
const std::string StretchRobot::RESOURCE_LIMITS{"package://robowflex_resources/stretch/config/"
                                                "joint_limits.yaml"};
const std::string  //
    StretchRobot::RESOURCE_KINEMATICS{"package://robowflex_resources/stretch/config/kinematics.yaml"};
const std::string  //
    OMPL::StretchOMPLPipelinePlanner::RESOURCE_CONFIG{"package://robowflex_resources/stretch/config/"
                                                      "ompl_planning.yaml"};

StretchRobot::StretchRobot() : Robot("stretch")
{
}

bool StretchRobot::initialize()
{
    bool success = false;

    // First attempt the `robowflex_resources` package, then attempt the "actual" resource files.
    if (IO::resolvePackage(RESOURCE_URDF).empty() or IO::resolvePackage(RESOURCE_SRDF).empty())
    {
        RBX_INFO("Initializing Stretch with `stretch_{description, moveit_config}`");
        success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);
    }
    else
    {
        RBX_INFO("Initializing Stretch with `robowflex_resources`");
        success = Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS, RESOURCE_KINEMATICS);
    }

    loadKinematics("stretch_arm");
    loadKinematics("stretch_gripper");
    loadKinematics("stretch_head");

    StretchRobot::openGripper();

    return success;
}

void StretchRobot::pointHead(const Eigen::Vector3d &point)
{
    const RobotPose point_pose = RobotPose(Eigen::Translation3d(point));
    const RobotPose point_pan = getLinkTF("link_head_pan").inverse() * point_pose;
    const RobotPose point_tilt = getLinkTF("link_head_tilt").inverse() * point_pose;

    const double pan = atan2(point_pan.translation().y(), point_pan.translation().x());
    const double tilt = -atan2(point_tilt.translation().z(),
                               hypot(point_tilt.translation().x(), point_tilt.translation().y()));

    const std::map<std::string, double> angles = {{"joint_head_pan", pan}, {"joint_head_tilt", tilt}};

    Robot::setState(angles);
}

void StretchRobot::openGripper()
{
    const std::map<std::string, double> angles = {{"joint_gripper_finger_left", 0.3},
                                                  {"joint_gripper_finger_right", 0.3}};

    Robot::setState(angles);
}

void StretchRobot::closeGripper()
{
    const std::map<std::string, double> angles = {{"joint_gripper_finger_left", 0.0},
                                                  {"joint_gripper_finger_right", 0.0}};

    Robot::setState(angles);
}

OMPL::StretchOMPLPipelinePlanner::StretchOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::StretchOMPLPipelinePlanner::initialize(const Settings &settings,
                                                  const std::vector<std::string> &adapters)
{
    if (IO::resolvePackage(RESOURCE_CONFIG).empty())
        return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
    return OMPLPipelinePlanner::initialize(RESOURCE_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}
