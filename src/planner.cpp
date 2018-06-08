#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "robowflex.h"

using namespace robowflex;

MotionRequestBuilder::MotionRequestBuilder(const Robot &robot, const std::string &group_name)
  : robot_(robot), group_name_(group_name), jmg_(robot.getModel()->getJointModelGroup(group_name))
{
    request_.group_name = group_name_;
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState start_state(robot_.getModel());
    start_state.setJointGroupPositions(jmg_, joints);

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
}

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState goal_state(robot_.getModel());
    goal_state.setJointGroupPositions(jmg_, joints);

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg_));
}

void MotionRequestBuilder::setGoalConfiguration(const std::string &ee_name,
                          const std::string &base_name,
                          const Eigen::Affine3d &pose,
                          const Geometry &geom,
                          const Eigen::Quaterniond &ee_orientation,
                          const Eigen::Vector3d angle_tolerances)
{
    moveit_msgs::Constraints constraints;

    moveit_msgs::PositionConstraint position;
    moveit_msgs::OrientationConstraint orientation;

    position.header.frame_id = base_name;
    position.link_name = ee_name;
    if(geom.isMesh()) {
      position.constraint_region.meshes.push_back(geom.getMeshMsg());
      position.constraint_region.mesh_poses.push_back(TF::poseEigenToMsg(pose));
    } else {
      position.constraint_region.primitives.push_back(geom.getSolidMsg());
      position.constraint_region.primitive_poses.push_back(TF::poseEigenToMsg(pose));
    }


    orientation.header.frame_id = base_name;
    orientation.link_name = ee_name;
    orientation.absolute_x_axis_tolerance = angle_tolerances[0];
    orientation.absolute_y_axis_tolerance = angle_tolerances[1];
    orientation.absolute_z_axis_tolerance = angle_tolerances[2];
    orientation.orientation = TF::quaternionEigenToMsg(ee_orientation);


    constraints.position_constraints.push_back(position);
    constraints.orientation_constraints.push_back(orientation);

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(constraints);
}

const planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequest()
{
    return request_;
}

planning_interface::MotionPlanResponse PipelinePlanner::plan(Scene &scene,
                                                             const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    if (pipeline_)
        pipeline_->generatePlan(scene.getScene(), request, response);

    return response;
}

const std::vector<std::string> OMPLPlanner::DEFAULT_ADAPTERS(
    {"default_planner_request_adapters/AddTimeParameterization",  // Defaults for OMPL Pipeline
     "default_planner_request_adapters/FixWorkspaceBounds",       //
     "default_planner_request_adapters/FixStartStateBounds",      //
     "default_planner_request_adapters/FixStartStateCollision",   //
     "default_planner_request_adapters/FixStartStatePathConstraints"});

void OMPLPlanner::OMPLSettings::setParam(IO::Handler &handler) const
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
    handler.setParam(prefix + "display_random_valid_states", display_random_valid_states);
    handler.setParam(prefix + "link_for_exploration_tree", link_for_exploration_tree);
    handler.setParam(prefix + "maximum_waypoint_distance", maximum_waypoint_distance);
}

OMPLPlanner::OMPLPlanner(Robot &robot) : PipelinePlanner(robot)
{
}

bool OMPLPlanner::initialize(const std::string &config_file, const OMPLSettings settings, const std::string &plugin,
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

    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_.getModel(), handler_.getHandle(), "planning_plugin",
                                                            "request_adapters"));

    return true;
}
