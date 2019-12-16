/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(fetch);

    ROS_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch.yml");

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create a motion planning request with a pose goal. Cube3
    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{0.4, 0.6, 0.92});

    Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};

    auto region = Geometry::makeSphere(0.01);
    auto tol = Eigen::Vector3d(0.1, 0.1 , 0.1);

    fetch->setFromIK(GROUP, region, pose, orn, tol);
    ROS_INFO("IK Calculated...");

    auto state = fetch->getState();
    rviz.visualizeState(state);
    return 0;
}
