#include <ros/ros.h>
#include <signal.h>

#include "robowflex.h"

void shutdown(int sig)
{
    ros::spinOnce();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);

    robowflex::Robot ur5("ur5");
    ur5.initialize("package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro",  // urdf
                   "package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf",     // srdf
                   "package://ur5_robotiq85_moveit_config/config/joint_limits.yaml",      // joint limits
                   "package://ur5_robotiq85_moveit_config/config/kinematics.yaml"         // kinematics
                   );

    ur5.loadOMPLPipeline("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
                         );

    // planning_interface::MotionPlanRequest req;
    // planning_interface::MotionPlanResponse res;
    // geometry_msgs::PoseStamped pose;
    // pose.header.frame_id = "torso_lift_link";
    // pose.pose.position.x = 0.75;
    // pose.pose.position.y = 0.0;
    // pose.pose.position.z = 0.0;
    // pose.pose.orientation.w = 1.0;

    // std::vector<double> tolerance_pose(3, 0.01);
    // std::vector<double> tolerance_angle(3, 0.01);

    // req.group_name = "right_arm";
    // moveit_msgs::Constraints pose_goal =
    //     kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);

    // req.goal_constraints.push_back(pose_goal);

    // // Now, call the pipeline and check whether planning was successful.
    // planning_pipeline->generatePlan(planning_scene, req, res);

    // robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    //   planning_scene->setCurrentState(response.trajectory_start);
    //   const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("right_arm");
    //   robot_state.setJointGroupPositions(joint_model_group,
    //   response.trajectory.joint_trajectory.points.back().positions);

    //   // Now, setup a joint space goal
    //   robot_state::RobotState goal_state(robot_model);
    //   std::vector<double> joint_values(7, 0.0);
    //   joint_values[0] = -2.0;
    //   joint_values[3] = -0.2;
    //   joint_values[5] = -0.15;
    //   goal_state.setJointGroupPositions(joint_model_group, joint_values);
    //   moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state,
    //   joint_model_group);

    //   req.goal_constraints.clear();
    //   req.goal_constraints.push_back(joint_goal);

    //   // Call the pipeline and visualize the trajectory
    //   planning_pipeline->generatePlan(planning_scene, req, res);

    ros::spin();
    return 0;
}
