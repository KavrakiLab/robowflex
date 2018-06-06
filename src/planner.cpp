#include <moveit/robot_model_loader/robot_model_loader.h>

#include "robowflex.h"

using namespace robowflex;

RobotDescription::RobotDescription(const std::string &description, const std::string &urdf_file,
                                   const std::string &srdf_file, const std::string &limits_file,
                                   const std::string &kinematics_file)
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
    loadYAMLParams(limits, description + "_planning");

    const YAML::Node &kinematics = loadFileToYAML(kinematics_file);
    loadYAMLParams(kinematics, description + "_kinematics");
}

RobotModel::RobotModel(const std::string &description)
{
    robot_model_loader::RobotModelLoader::Options options(description);
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    robot_ = model_loader.getModel();
}

Planner::Planner()
{
}

MoveItPlanner::MoveItPlanner() : Planner()
{
}
