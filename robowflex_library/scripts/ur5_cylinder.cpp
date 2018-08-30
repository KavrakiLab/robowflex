/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(ur5);

    ROS_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Create the cylinder we want to grasp
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});

    auto cylinder = Geometry::makeCylinder(0.025, 0.1);

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);
    scene->updateCollisionObject("cylinder", cylinder, pose);

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    request.addCylinderSideGrasp("ee_link", "world",  //
                                 pose, cylinder,      //
                                 0.15, 0.04, 16);      //

    rviz.addGoalMarker("goal", request);                        // Visualize the grasping regions
    rviz.updateMarkers();

    ROS_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    ROS_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    rviz.removeMarker("goal");
    rviz.removeMarker("cylinder");
    rviz.updateMarkers();

    rviz.removeScene();

    ROS_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
