/* Author: Zachary Kingston */

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
    setSRDFPostProcessFunction(std::bind(&FetchRobot::addVirtualJointSRDF, this, std::placeholders::_1));

    bool success = Robot::initialize(URDF, SRDF, LIMITS, KINEMATICS);
    loadKinematics("arm");
    loadKinematics("arm_with_torso");

    return true;
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
