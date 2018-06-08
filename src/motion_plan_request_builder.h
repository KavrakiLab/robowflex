#ifndef MOTION_PLAN_REQUEST_BUILDER_CPP
#define MOTION_PLAN_REQUEST_BUILDER_CPP

#include <ros/ros.h>
#include <signal.h>

#include <moveit/kinematic_constraints/utils.h>

#include "robowflex.h"

namespace robowflex
{
    class MotionRequestBuilder
    {
        const Robot &robot;
        const robot_model::JointModelGroup *jmg;
        std::string group_name;

    public:
        robot_state::RobotState &start_state;

        planning_interface::MotionPlanRequest request;

        MotionRequestBuilder(const Robot &robot, const std::string group_name, robot_state::RobotState &start_state)
          : robot(robot), group_name(group_name), start_state(start_state)
        {
            jmg = start_state.getJointModelGroup(group_name);
        }

        planning_interface::MotionPlanRequest buildRequest(const moveit_msgs::Constraints joint_goal)
        {
            planning_interface::MotionPlanRequest req;
            req.group_name = group_name;
            req.goal_constraints.push_back(joint_goal);
            return req;
        }
    };
}  // namespace robowflex

#endif
