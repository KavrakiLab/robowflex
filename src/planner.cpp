#include <functional>
#include <numeric>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include "robowflex.h"

using namespace robowflex;

const std::vector<std::string> Robot::DEFAULT_ADAPTERS(
    {"default_planner_request_adapters/AddTimeParameterization", "default_planner_request_adapters/FixWorkspaceBounds",
     "default_planner_request_adapters/FixStartStateBounds", "default_planner_request_adapters/FixStartStateCollision",
     "default_planner_request_adapters/FixStartStatePathConstraints"});

Robot::Robot(const std::string &name) : name_(name), nh_(name_)
{
}

bool Robot::initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                       const std::string &kinematics_file)
{
    if (!loadRobotDescription(urdf_file, srdf_file, limits_file, kinematics_file))
        return false;

    model_ = std::move(loadRobotModel());
    scene_.reset(new planning_scene::PlanningScene(model_));
    return true;
}

bool Robot::loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                 const std::string &limits_file, const std::string &kinematics_file)
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

    nh_.setParam("robot_description", urdf_string);
    nh_.setParam("robot_description_semantic", srdf_string);
    loadYAMLtoROS(limits.second, "robot_description_planning", nh_);
    loadYAMLtoROS(kinematics.second, "robot_description_kinematics", nh_);

    return true;
}

robot_model::RobotModelPtr Robot::loadRobotModel()
{
    robot_model_loader::RobotModelLoader::Options options(name_ + "/robot_description");
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    return model_loader.getModel();
}

void Robot::loadOMPLPipeline(const std::string &config_file, const std::string &plugin,
                             const std::vector<std::string> &adapters)
{
    nh_.setParam("planning_plugin", plugin);

    std::stringstream ss;
    for (std::size_t i = 0; i < adapters.size(); ++i)
    {
        ss << adapters[i];
        if (i < adapters.size() - 1)
            ss << " ";
    }

    nh_.setParam("request_adapters", ss.str());

    pipeline_.reset(new planning_pipeline::PlanningPipeline(model_, nh_, "planning_plugin", "request_adapters"));
}
