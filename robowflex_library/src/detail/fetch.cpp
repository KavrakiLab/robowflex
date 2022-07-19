/* Author: Zachary Kingston */

#include <cmath>

#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string FetchRobot::DEFAULT_URDF{"package://fetch_description/robots/fetch.urdf"};
const std::string FetchRobot::DEFAULT_SRDF{"package://fetch_moveit_config/config/fetch.srdf"};
const std::string FetchRobot::DEFAULT_LIMITS{"package://fetch_moveit_config/config/joint_limits.yaml"};
const std::string FetchRobot::DEFAULT_KINEMATICS{"package://fetch_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::FetchOMPLPipelinePlanner::DEFAULT_CONFIG{"package://fetch_moveit_config/config/ompl_planning.yaml"};

const std::string FetchRobot::RESOURCE_URDF{"package://robowflex_resources/fetch/robots/fetch.urdf"};
const std::string FetchRobot::RESOURCE_SRDF{"package://robowflex_resources/fetch/config/fetch.srdf"};
const std::string FetchRobot::RESOURCE_LIMITS{"package://robowflex_resources/fetch/config/joint_limits.yaml"};
const std::string  //
    FetchRobot::RESOURCE_LIMITS_LOW{"package://robowflex_resources/fetch/config/joint_limits_low.yaml"};
const std::string  //
    FetchRobot::RESOURCE_KINEMATICS{"package://robowflex_resources/fetch/config/kinematics.yaml"};
const std::string  //
    OMPL::FetchOMPLPipelinePlanner::RESOURCE_CONFIG{"package://robowflex_resources/fetch/config/"
                                                    "ompl_planning.yaml"};

FetchRobot::FetchRobot() : Robot("fetch")
{
}

bool FetchRobot::initialize(bool addVirtual, bool use_low_limits)
{
    if (addVirtual)
        setSRDFPostProcessAddPlanarJoint("base_joint");

    setURDFPostProcessFunction([this](tinyxml2::XMLDocument &doc) { return addCastersURDF(doc); });

    bool success = false;
    // First attempt the `robowflex_resources` package, then attempt the "actual" resource files.
    if (IO::resolvePackage(RESOURCE_URDF).empty() or IO::resolvePackage(RESOURCE_SRDF).empty())
    {
        RBX_INFO("Initializing Fetch with `fetch_{description, moveit_config}`");
        success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);
    }
    else
    {
        RBX_INFO("Initializing Fetch with `robowflex_resources`");
        if (use_low_limits)
            success =
                Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS_LOW, RESOURCE_KINEMATICS);
        else
            success = Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS, RESOURCE_KINEMATICS);
    }

    loadKinematics("arm");
    loadKinematics("arm_with_torso");

    FetchRobot::openGripper();

    return success;
}

bool FetchRobot::addCastersURDF(tinyxml2::XMLDocument &doc)
{
    for (const auto &name : {"bl_caster", "br_caster", "fl_caster", "fr_caster"})
    {
        auto link_name = std::string(name) + "_link";
        if (not isLinkURDF(doc, link_name))
        {
            tinyxml2::XMLElement *caster_link = doc.NewElement("link");
            caster_link->SetAttribute("name", link_name.c_str());
            doc.FirstChildElement("robot")->InsertFirstChild(caster_link);

            auto joint_name = std::string(name) + "_joint";
            tinyxml2::XMLElement *caster_joint = doc.NewElement("joint");
            caster_joint->SetAttribute("name", joint_name.c_str());
            caster_joint->SetAttribute("type", "fixed");

            tinyxml2::XMLElement *parent = doc.NewElement("parent");
            parent->SetAttribute("link", "base_link");
            caster_joint->InsertFirstChild(parent);

            tinyxml2::XMLElement *child = doc.NewElement("child");
            child->SetAttribute("link", link_name.c_str());
            caster_joint->InsertFirstChild(child);

            doc.FirstChildElement("robot")->InsertFirstChild(caster_joint);
        }
    }

    return true;
}

void FetchRobot::pointHead(const Eigen::Vector3d &point)
{
    const RobotPose point_pose = RobotPose(Eigen::Translation3d(point));
    const RobotPose point_pan = getLinkTF("head_pan_link").inverse() * point_pose;
    const RobotPose point_tilt = getLinkTF("head_tilt_link").inverse() * point_pose;

    const double pan = atan2(point_pan.translation().y(), point_pan.translation().x());
    const double tilt = -atan2(point_tilt.translation().z(),
                               hypot(point_tilt.translation().x(), point_tilt.translation().y()));

    const std::map<std::string, double> angles = {{"head_pan_joint", pan}, {"head_tilt_joint", tilt}};

    Robot::setState(angles);
}

void FetchRobot::openGripper()
{
    const std::map<std::string, double> angles = {{"l_gripper_finger_joint", 0.04},
                                                  {"r_gripper_finger_joint", 0.04}};

    Robot::setState(angles);
}

void FetchRobot::closeGripper()
{
    const std::map<std::string, double> angles = {{"l_gripper_finger_joint", 0.0},
                                                  {"r_gripper_finger_joint", 0.0}};

    Robot::setState(angles);
}

void FetchRobot::setBasePose(double x, double y, double theta)
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

OMPL::FetchOMPLPipelinePlanner::FetchOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::FetchOMPLPipelinePlanner::initialize(const Settings &settings,
                                                const std::vector<std::string> &adapters)
{
    if (IO::resolvePackage(RESOURCE_CONFIG).empty())
        return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
    return OMPLPipelinePlanner::initialize(RESOURCE_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}
