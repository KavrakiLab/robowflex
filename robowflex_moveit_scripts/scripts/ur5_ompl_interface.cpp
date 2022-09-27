/* Author: Zachary Kingston */

#include <robowflex_moveit/core/builder.h>
#include <robowflex_moveit/robots/ur5.h>
#include <robowflex_moveit/core/geometry.h>
#include <robowflex_moveit/core/planning.h>
#include <robowflex_moveit/core/robot.h>
#include <robowflex_moveit/core/scene.h>
#include <robowflex_moveit/io/ros.h>

#include <robowflex_moveit/core/ompl_interface.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);

    // Create an OMPL interface planner for the ur5.
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(ur5);
    planner->initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml");

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    auto sphere = std::make_shared<Geometry>(Geometry::ShapeType::SPHERE, Eigen::Vector3d{0.01, 0, 0});
    request.setGoalRegion("ee_link", "world",      // links
                          pose, sphere,            // position
                          orn, {0.01, 0.01, 0.01}  // orientation
    );

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
