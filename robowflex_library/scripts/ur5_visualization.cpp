/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/io/visualization.h>
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

    ROS_INFO("Press enter to continue...");
    std::cin.get();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);
    scene->fromYAMLFile("package://robowflex_library/yaml/test.yml");

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    auto region = Geometry::makeSphere(0.1);

    rviz.addGeometryMarker("goal", region, "/map", pose);
    rviz.updateMarkers();

    request.setGoalRegion("ee_link", "world",      // links
                          pose, region,            // position
                          orn, {0.01, 0.01, 0.01}  // orientation
    );

    ROS_INFO("Press enter to continue...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    ROS_INFO("Press enter to continue...");
    std::cin.get();

    rviz.removeMarker("goal");
    rviz.updateMarkers();

    ROS_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
