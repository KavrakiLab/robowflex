/* Author: Zachary Kingston */

#include <cmath>

#include <robowflex_library/log.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>

using namespace robowflex;

const std::string FetchRobot::URDF{"package://fetch_description/robots/fetch.urdf"};
const std::string FetchRobot::SRDF{"package://fetch_moveit_config/config/fetch.srdf"};
const std::string FetchRobot::LIMITS{"package://fetch_moveit_config/config/joint_limits.yaml"};
const std::string FetchRobot::KINEMATICS{"package://fetch_moveit_config/config/kinematics.yaml"};

const std::string  //
    OMPL::FetchOMPLPipelinePlanner::CONFIG{"package://fetch_moveit_config/config/ompl_planning.yaml"};

FetchRobot::FetchRobot() : Robot("fetch")
{
}

bool FetchRobot::initialize(bool addVirtual)
{
    if (addVirtual)
        setSRDFPostProcessFunction([this](tinyxml2::XMLDocument &doc) { return addVirtualJointSRDF(doc); });

    setURDFPostProcessFunction([this](tinyxml2::XMLDocument &doc) { return addCastersURDF(doc); });

    bool success = Robot::initialize(URDF, SRDF, LIMITS, KINEMATICS);
    loadKinematics("arm");
    loadKinematics("arm_with_torso");

    FetchRobot::openGripper();

    return success;
}

bool FetchRobot::addVirtualJointSRDF(tinyxml2::XMLDocument &doc)
{
    tinyxml2::XMLElement *virtual_joint = doc.NewElement("virtual_joint");
    virtual_joint->SetAttribute("name", "base_joint");
    virtual_joint->SetAttribute("type", "planar");
    virtual_joint->SetAttribute("parent_frame", "world");
    virtual_joint->SetAttribute("child_link", "base_link");

    doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

    return true;
}

bool FetchRobot::addCastersURDF(tinyxml2::XMLDocument &doc)
{
    for (const auto name : {"bl_caster", "br_caster", "fl_caster", "fr_caster"})
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

bool OMPL::FetchOMPLPipelinePlanner::initialize(const Settings &settings, const std::string &config_file,
                                                const std::string &plugin,
                                                const std::vector<std::string> &adapters)
{
    return OMPLPipelinePlanner::initialize(config_file, settings, plugin, adapters);
}
