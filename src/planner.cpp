#include <functional>
#include <numeric>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include "robowflex.h"

using namespace robowflex;

const std::vector<std::string>
    Robot::DEFAULT_ADAPTERS({"default_planner_request_adapters/AddTimeParameterization",  // Defaults for OMPL Pipeline
                             "default_planner_request_adapters/FixWorkspaceBounds",       //
                             "default_planner_request_adapters/FixStartStateBounds",      //
                             "default_planner_request_adapters/FixStartStateCollision",   //
                             "default_planner_request_adapters/FixStartStatePathConstraints"});

Robot::Robot(const std::string &name) : name_(name), handler_(name_)
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

robot_model::RobotModelPtr Robot::loadRobotModel()
{
    robot_model_loader::RobotModelLoader::Options options(name_ + "/robot_description");
    options.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader model_loader(options);
    return model_loader.getModel();
}

void Robot::OMPLSettings::setParam(IO::Handler &handler) const
{
    const std::string prefix = "ompl/";
    handler.setParam(prefix + "max_goal_samples", max_goal_samples);
    handler.setParam(prefix + "max_goal_sampling_attempts", max_goal_sampling_attempts);
    handler.setParam(prefix + "max_planning_threads", max_planning_threads);
    handler.setParam(prefix + "max_solution_segment_length", max_solution_segment_length);
    handler.setParam(prefix + "max_state_sampling_attempts", max_state_sampling_attempts);
    handler.setParam(prefix + "minimum_waypoint_count", minimum_waypoint_count);
    handler.setParam(prefix + "simplify_solutions", simplify_solutions);
    handler.setParam(prefix + "use_constraints_approximations", use_constraints_approximations);
}

void Robot::loadOMPLPipeline(const std::string &config_file, const OMPLSettings settings, const std::string &plugin,
                             const std::vector<std::string> &adapters)
{
    handler_.setParam("planning_plugin", plugin);

    std::stringstream ss;
    for (std::size_t i = 0; i < adapters.size(); ++i)
    {
        ss << adapters[i];
        if (i < adapters.size() - 1)
            ss << " ";
    }

    handler_.setParam("request_adapters", ss.str());
    settings.setParam(handler_);

    pipeline_.reset(
        new planning_pipeline::PlanningPipeline(model_, handler_.getHandle(), "planning_plugin", "request_adapters"));
}
