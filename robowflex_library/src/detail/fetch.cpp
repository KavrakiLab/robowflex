/* Author: Zachary Kingston */

#include <tinyxml2.h>
#include <robowflex_library/io.h>
#include <robowflex_library/detail/fetch.h>

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

bool FetchRobot::initialize()
{
    bool success = loadXMLFile(ROBOT_DESCRIPTION, URDF)                                // urdf
                   && loadSRDFFile()                                                   // srdf
                   && loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_PLANNING, LIMITS)         // joint limits
                   && loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_KINEMATICS, KINEMATICS);  // kinematics

    loadRobotModel();

    loadKinematics("arm");
    loadKinematics("arm_with_torso");

    return true;
}

bool FetchRobot::loadSRDFFile()
{
    const std::string string = IO::loadXMLToString(SRDF);
    if (string.empty())
    {
        ROS_ERROR("Failed to load XML file `%s`.", SRDF.c_str());
        return false;
    }

    tinyxml2::XMLDocument doc;
    doc.Parse(string.c_str());

    tinyxml2::XMLElement *virtual_joint = doc.NewElement("virtual_joint");
    virtual_joint->SetAttribute("name", "base_joint");
    virtual_joint->SetAttribute("type", "planar");
    virtual_joint->SetAttribute("parent_frame", "world");
    virtual_joint->SetAttribute("child_link", "base_link");

    doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);

    handler_.setParam(ROBOT_DESCRIPTION + ROBOT_SEMANTIC, std::string(printer.CStr()));
    return true;
}

void FetchRobot::setBasePose(double x, double y, double theta)
{
    std::map<std::string, double> pose = {
        {"base_joint/x", x}, {"base_joint/y", y}, {"base_joint/theta", theta}};

    scratch_->setVariablePositions(pose);
    scratch_->update();
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
