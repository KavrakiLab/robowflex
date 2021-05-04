/* Author: Carlos Quintero Pena */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/trajectory.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Create an empty scene and add a cylinder to it
    auto scene = std::make_shared<Scene>(ur5);
    auto cylinder_position = Eigen::Vector3d{-0.270, 0.42, 1.1572};
    RobotPose original_pose = RobotPose::Identity();
    original_pose.translate(cylinder_position);
    original_pose.rotate(Eigen::Quaterniond{0.707, 0.0, 0.707, 0.0});
    scene->updateCollisionObject("cylinder", Geometry::makeCylinder(0.025, 0.1), original_pose);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    // Set start configuration to the robot scratch state and attach the cylinder to it
    ur5->setState({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    scene->attachObject(*ur5->getScratchState(), "cylinder");

    // Shift the original end effector pose
    auto shift = Eigen::Vector3d{0.0, -0.5, 0.0};
    RobotPose goal_pose = scene->getCurrentStateConst().getFrameTransform("ee_link");
    goal_pose.translate(shift);

    // Theoretical new pose for the cylinder
    auto th_new_pose = cylinder_position + shift;

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, "manipulator"));
    request->setStartConfiguration(std::make_shared<robot_state::RobotState>(scene->getCurrentStateConst()));
    request->setGoalRegion("ee_link", "world", goal_pose, Geometry::makeSphere(0.01),
                           Eigen::Quaterniond{goal_pose.rotation()}, {0.001, 0.001, 0.001});

    // Do motion planning!
    RBX_INFO("Planning");
    planning_interface::MotionPlanResponse res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    RBX_INFO("Planning worked!");

    // Detach the object
    scene->getCurrentState() = *res.trajectory_->getLastWayPointPtr();
    scene->detachObject("cylinder");

    // Get new position for the cylinder and compare to its theoretical value.
    auto new_pose = scene->getObjectPose("cylinder");
    auto diff = th_new_pose - new_pose.translation();
    if (diff.norm() < 0.01)
        RBX_INFO("Detach works! Distance between positions %f", diff.norm());
    else
        RBX_ERROR("Detach does not work! Distance between positions %f", diff.norm());

    return 0;
}
