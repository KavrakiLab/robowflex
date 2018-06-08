#include <ros/ros.h>
#include <signal.h>

#include <moveit/kinematic_constraints/utils.h>

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

    robowflex::Scene scene(ur5);

    robowflex::OMPLPlanner planner(ur5);
    planner.initialize();  // Don't need to by default
    // planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    //                    );

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

    robot_state::RobotState &start_state = scene.getCurrentState();

    const robot_model::JointModelGroup *jmg = start_state.getJointModelGroup("manipulator");
    start_state.setJointGroupPositions(jmg, {0, 0, 0, 0, 0, 0});

    robot_state::RobotState goal_state(ur5.getModel());
    goal_state.setJointGroupPositions(jmg, {-0.39, -0.69, -2.12, 2.82, -0.39, 0});
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);

    robowflex::MotionRequestBuilder my_req_builder(ur5, "manipulator", start_state);
    planning_interface::MotionPlanRequest request = my_req_builder.buildRequest(joint_goal);

    ros::Rate rate(1);
    while (ros::ok())
    {
        planning_interface::MotionPlanResponse res = planner.plan(scene, request);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
