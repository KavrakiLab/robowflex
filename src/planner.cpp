#include <moveit/robot_model_loader/robot_model_loader.h>

#include "robowflex.h"

using namespace robowflex;

void robowflex::loadRobotDescription(const std::string &description, const std::string &urdf_file, const std::string &srdf_file,
                          const std::string &limits_file, const std::string &kinematics_file)
{
    const std::string urdf_string = loadFileToXML(urdf_file);
    const std::string srdf_string = loadFileToXML(srdf_file);
    ros::param::set(description, urdf_string);
    ros::param::set(description + "_semantic", srdf_string);

    if (urdf_string.empty() || srdf_string.empty())
    {
        ROS_ERROR("Failed to load URDF or SRDF file.");
    }

    const YAML::Node &limits = loadFileToYAML(limits_file);
    loadYAMLtoROS(limits, description + "_planning");

    const YAML::Node &kinematics = loadFileToYAML(kinematics_file);
    loadYAMLtoROS(kinematics, description + "_kinematics");
}

robot_model::RobotModelPtr robowflex::loadRobotModel(const std::string &description)
{
    robot_model_loader::RobotModelLoader::Options options(description);
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    return model_loader.getModel();
}
