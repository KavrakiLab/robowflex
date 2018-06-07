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
    const std::string urdf_string = IO::loadFileToXML(urdf_file);
    if (urdf_string.empty())
    {
        ROS_ERROR("Failed to load URDF.");
        return false;
    }

    const std::string srdf_string = IO::loadFileToXML(srdf_file);
    if (srdf_string.empty())
    {
        ROS_ERROR("Failed to load SRDF.");
        return false;
    }

    auto &limits = IO::loadFileToYAML(limits_file);
    if (!limits.first)
    {
        ROS_ERROR("Failed to load joint limits.");
        return false;
    }

    auto &kinematics = IO::loadFileToYAML(kinematics_file);
    if (!kinematics.first)
    {
        ROS_ERROR("Failed to load kinematics.");
        return false;
    }

    handler_.setParam("robot_description", urdf_string);
    handler_.setParam("robot_description_semantic", srdf_string);
    handler_.loadYAMLtoROS(limits.second, "robot_description_planning");
    handler_.loadYAMLtoROS(kinematics.second, "robot_description_kinematics");

    return true;
}

void Robot::loadRobotModel()
{
    robot_model_loader::RobotModelLoader::Options options(name_ + "/robot_description");
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    model_ = std::move(model_loader.getModel());
}

