#include <moveit/robot_model_loader/robot_model_loader.h>

#include "robowflex.h"

using namespace robowflex;

Robot::Robot(const std::string &name) : name_(name), handler_(name_)
{
}

bool Robot::initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                       const std::string &kinematics_file)
{
    if (!loadRobotDescription(urdf_file, srdf_file, limits_file, kinematics_file))
        return false;

    loadRobotModel();
    return true;
}

bool Robot::loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                 const std::string &limits_file, const std::string &kinematics_file)
{
    if (!handler_.hasParam("robot_description"))
    {
        const std::string urdf_string = IO::loadXMLToString(urdf_file);
        if (urdf_string.empty())
        {
            ROS_ERROR("Failed to load URDF.");
            return false;
        }
        handler_.setParam("robot_description", urdf_string);
    }

    if (!handler_.hasParam("robot_description_semantic"))
    {
        const std::string srdf_string = IO::loadXMLToString(srdf_file);
        if (srdf_string.empty())
        {
            ROS_ERROR("Failed to load SRDF.");
            return false;
        }
        handler_.setParam("robot_description_semantic", srdf_string);
    }

    if (!handler_.hasParam("robot_description_planning"))
    {
        auto &limits = IO::loadFileToYAML(limits_file);
        if (!limits.first)
        {
            ROS_ERROR("Failed to load joint limits.");
            return false;
        }
        handler_.loadYAMLtoROS(limits.second, "robot_description_planning");
    }

    if (!handler_.hasParam("robot_description_kinematics"))
    {
        auto &kinematics = IO::loadFileToYAML(kinematics_file);
        if (!kinematics.first)
        {
            ROS_ERROR("Failed to load kinematics.");
            return false;
        }
        handler_.loadYAMLtoROS(kinematics.second, "robot_description_kinematics");
    }

    return true;
}

void Robot::loadRobotModel()
{
    robot_model_loader::RobotModelLoader::Options options(handler_.getNamespace() + "/robot_description");
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    model_ = std::move(model_loader.getModel());
}
