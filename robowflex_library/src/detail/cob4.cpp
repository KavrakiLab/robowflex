/* Author: Juan D. Hernandez */

#include <cmath>

#include <robowflex_library/log.h>
#include <robowflex_library/detail/cob4.h>
#include <robowflex_library/io.h>

using namespace robowflex;

const std::string  //
    Cob4Robot::DEFAULT_URDF{"package://cob_hardware_config/robots/cob4-8/urdf/cob4-8.urdf.xacro"};
const std::string  //
    Cob4Robot::DEFAULT_SRDF{"package://cob_moveit_config/robots/cob4-8/moveit/config/cob4-8.srdf"};
const std::string  //
    Cob4Robot::DEFAULT_LIMITS{"package://cob_moveit_config/robots/cob4-8/moveit/config/joint_limits.yaml"};
const std::string  //
    Cob4Robot::DEFAULT_KINEMATICS{"package://cob_moveit_config/robots/cob4-8/moveit/config/kinematics.yaml"};
const std::string  //
    OMPL::Cob4OMPLPipelinePlanner::DEFAULT_CONFIG{"package://cob_moveit_config/robots/cob4-8/moveit/config/"
                                                  "ompl_planning.yaml"};

const std::string Cob4Robot::RESOURCE_URDF{"package://robowflex_resources/cob/robots/cob4-8.urdf.xacro"};
const std::string Cob4Robot::RESOURCE_SRDF{"package://robowflex_resources/cob/config/cob4-8.srdf"};
const std::string Cob4Robot::RESOURCE_LIMITS{"package://robowflex_resources/cob/config/joint_limits.yaml"};
const std::string  //
    Cob4Robot::RESOURCE_KINEMATICS{"package://robowflex_resources/cob/config/kinematics.yaml"};
const std::string  //
    OMPL::Cob4OMPLPipelinePlanner::RESOURCE_CONFIG{"package://robowflex_resources/cob/config/"
                                                   "ompl_planning.yaml"};

Cob4Robot::Cob4Robot() : Robot("cob4")
{
}

bool Cob4Robot::initialize()
{
    bool success = false;

    // First attempt the `robowflex_resources` package, then attempt the "actual" resource files.
    if (IO::resolvePackage(RESOURCE_URDF).empty() or IO::resolvePackage(RESOURCE_SRDF).empty())
    {
        RBX_INFO("Initializing Cob4 with `cob4_{description, moveit_config}`");
        success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);
    }
    else
    {
        RBX_INFO("Initializing Cob4 with `robowflex_resources`");
        success = Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS, RESOURCE_KINEMATICS);
    }

    loadKinematics("arm_left");
    loadKinematics("arm_right");

    Cob4Robot::openGrippers();

    return success;
}

void Cob4Robot::pointHead(const Eigen::Vector3d &point)
{
    const RobotPose point_pose = RobotPose(Eigen::Translation3d(point));
    const RobotPose point_pan = getLinkTF("head_3_link").inverse() * point_pose;

    const double pan = atan2(point_pan.translation().y(), point_pan.translation().x());

    const std::map<std::string, double> angles = {{"head_1_joint", pan}};

    Robot::setState(angles);
}

void Cob4Robot::openGrippers()
{
    const std::map<std::string, double> angles = {{"gripper_left_finger_1_joint", -1.0},
                                                  {"gripper_left_finger_2_joint", 1.2},
                                                  {"gripper_right_finger_1_joint", -1.0},
                                                  {"gripper_right_finger_2_joint", 1.2}};

    Robot::setState(angles);
}

void Cob4Robot::openLeftGripper()
{
    const std::map<std::string, double> angles = {{"gripper_left_finger_1_joint", -1.0},
                                                  {"gripper_left_finger_2_joint", 1.2}};

    Robot::setState(angles);
}

void Cob4Robot::openRightGripper()
{
    const std::map<std::string, double> angles = {{"gripper_right_finger_1_joint", -1.0},
                                                  {"gripper_right_finger_2_joint", 1.2}};

    Robot::setState(angles);
}

void Cob4Robot::closeGrippers()
{
    const std::map<std::string, double> angles = {{"l_gripper_finger_joint", 0.0},
                                                  {"r_gripper_finger_joint", 0.0}};

    Robot::setState(angles);
}

void Cob4Robot::closeLeftGripper()
{
    const std::map<std::string, double> angles = {{"gripper_left_finger_1_joint", 0.0},
                                                  {"gripper_left_finger_2_joint", 0.0}};

    Robot::setState(angles);
}

void Cob4Robot::closeRightGripper()
{
    const std::map<std::string, double> angles = {{"gripper_right_finger_1_joint", 0.0},
                                                  {"gripper_right_finger_2_joint", 0.0}};

    Robot::setState(angles);
}

void Cob4Robot::setBasePose(double x, double y, double theta)
{
    if (hasJoint("base_joint/x") && hasJoint("base_joint/y") && hasJoint("base_joint/theta"))
    {
        const std::map<std::string, double> pose = {
            {"base_joint/x", x}, {"base_joint/y", y}, {"base_joint/theta", theta}};

        scratch_->setVariablePositions(pose);
        scratch_->update();
    }
    else
        RBX_WARN("base_joint does not exist, cannot move base! You need to set addVirtual to true");
}

OMPL::Cob4OMPLPipelinePlanner::Cob4OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::Cob4OMPLPipelinePlanner::initialize(const Settings &settings,
                                               const std::vector<std::string> &adapters)
{
    if (IO::resolvePackage(RESOURCE_CONFIG).empty())
        return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
    else
        return OMPLPipelinePlanner::initialize(RESOURCE_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}
