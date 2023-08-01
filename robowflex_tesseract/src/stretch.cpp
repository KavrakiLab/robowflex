/* Author: Carlos Quintero-Peña */

#include <cmath>

#include <robowflex_tesseract/stretch.h>
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

void StretchRobot::setSRDFPostProcessAddMobileManipulatorGroup(const std::string &base_group,
                                                        const std::string &manip_group,
                                                        const std::string &base_manip_group)
{
    setSRDFPostProcessFunction(
        [&, base_group, manip_group, base_manip_group](tinyxml2::XMLDocument &doc) -> bool {
            // Add mobile base joint.
            const std::string &base_joint_name = "base_joint";
            tinyxml2::XMLElement *virtual_joint = doc.NewElement("virtual_joint");
            virtual_joint->SetAttribute("name", base_joint_name.c_str());
            virtual_joint->SetAttribute("type", "planar");
            virtual_joint->SetAttribute("parent_frame", "world");
            virtual_joint->SetAttribute("child_link", model_->getRootLink()->getName().c_str());
            doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

            // Add mobile base group using the created joint.
            tinyxml2::XMLElement *group_base = doc.NewElement("group");
            group_base->SetAttribute("name", base_group.c_str());
            tinyxml2::XMLElement *base_joint = doc.NewElement("joint");
            base_joint->SetAttribute("name", base_joint_name.c_str());
            group_base->InsertFirstChild(base_joint);
            doc.FirstChildElement("robot")->InsertFirstChild(group_base);

            // Add mobile base manipulator group.
            tinyxml2::XMLElement *base_manip = doc.NewElement("group");
            base_manip->SetAttribute("name", base_manip_group.c_str());
            tinyxml2::XMLElement *manip = doc.NewElement("group");
            manip->SetAttribute("name", manip_group.c_str());
            base_manip->InsertFirstChild(manip);
            tinyxml2::XMLElement *base = doc.NewElement("group");
            base->SetAttribute("name", base_group.c_str());
            base_manip->InsertFirstChild(base);
            doc.FirstChildElement("robot")->InsertFirstChild(base_manip);

            return true;
        });
}

void StretchRobot::setKinematicsPostProcessAddBaseManipulatorPlugin(const std::string &base_group,
                                                             const std::string &base_manip_group,
                                                             double search_resolution, double timeout)
{
    setKinematicsPostProcessFunction(
        [&, base_group, base_manip_group, search_resolution, timeout](YAML::Node &node) -> bool {
            YAML::Node ksb_node;
            ksb_node["kinematics_solver"] = "base_manipulator_kinematics_plugin/"
                                            "BaseManipulatorKinematicsPlugin";
            ksb_node["kinematics_solver_search_resolution"] = search_resolution;
            ksb_node["kinematics_solver_timeout"] = timeout;

            node[base_group.c_str()] = ksb_node;

            YAML::Node ks_node;
            ks_node["kinematics_solver"] = "base_manipulator_kinematics_plugin/"
                                           "BaseManipulatorKinematicsPlugin";
            ks_node["kinematics_solver_search_resolution"] = search_resolution;
            ks_node["kinematics_solver_timeout"] = timeout;

            node[base_manip_group.c_str()] = ks_node;

            return true;
        });
}

bool StretchRobot::initialize(bool addVirtual, bool addBaseManip, const std::string &mob_base_manip, const std::string &manip)
{
    if (addBaseManip)
    {
        // const std::string &mob_base_manip = "mobile_base_manipulator";
        const std::string &mobile_base = "mobile_base";
        setSRDFPostProcessAddMobileManipulatorGroup(mobile_base, manip, mob_base_manip);
        setKinematicsPostProcessAddBaseManipulatorPlugin(mobile_base, mob_base_manip);
    }
    else if (addVirtual)
        setSRDFPostProcessAddPlanarJoint("base_joint");

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
    if (addBaseManip)
        loadKinematics("mobile_base_manipulator");

    StretchRobot::openGripper();

    // Lift up the arm to avoid self-collisions.
    setState(std::map<std::string, double>{{"joint_lift", 0.2}});

    return success;
}

void StretchRobot::setBasePose(double x, double y, double theta)
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

void StretchRobot::pointHead(const Eigen::Vector3d &point)
{
    const RobotPose point_pose = RobotPose(Eigen::Translation3d(point));
    const RobotPose point_pan = getLinkTF("link_head_pan").inverse() * point_pose;
    const RobotPose point_tilt = getLinkTF("link_head_tilt").inverse() * point_pose;

    const double pan = atan2(point_pan.translation().y(), point_pan.translation().x());
    // const double tilt = -atan2(point_tilt.translation().z(),
                               // hypot(point_tilt.translation().x(), point_tilt.translation().y()));
    // const double tilt = atan2(point_tilt.translation().z(),
                               // hypot(point_tilt.translation().x(), point_tilt.translation().y()));

    const double tilt = atan2(point_tilt.translation().z(),
                               hypot(point_tilt.translation().x(), point_tilt.translation().y()));
    std::cout << "pan inside " << pan << ", tilt inside: " << tilt << std::endl;

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
