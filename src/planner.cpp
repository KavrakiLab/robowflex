#include <moveit/robot_model_loader/robot_model_loader.h>

#include "robowflex.h"

using namespace robowflex;

bool robowflex::loadRobotDescription(const std::string &description, const std::string &urdf_file,
                                     const std::string &srdf_file, const std::string &limits_file,
                                     const std::string &kinematics_file)
{
    const std::string urdf_string = loadFileToXML(urdf_file);
    if (urdf_string.empty())
    {
        ROS_ERROR("Failed to load URDF.");
        return false;
    }

    const std::string srdf_string = loadFileToXML(srdf_file);
    if (srdf_string.empty())
    {
        ROS_ERROR("Failed to load SRDF.");
        return false;
    }

    auto &limits = loadFileToYAML(limits_file);
    if (!limits.first)
    {
        ROS_ERROR("Failed to load joint limits.");
        return false;
    }

    auto &kinematics = loadFileToYAML(kinematics_file);
    if (!kinematics.first)
    {
        ROS_ERROR("Failed to load kinematics.");
        return false;
    }

    ros::param::set(description, urdf_string);
    ros::param::set(description + "_semantic", srdf_string);

    loadYAMLtoROS(limits.second, description + "_planning");
    loadYAMLtoROS(kinematics.second, description + "_kinematics");

    return true;
}

robot_model::RobotModelPtr robowflex::loadRobotModel(const std::string &description)
{
    robot_model_loader::RobotModelLoader::Options options(description);
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    return model_loader.getModel();
}
