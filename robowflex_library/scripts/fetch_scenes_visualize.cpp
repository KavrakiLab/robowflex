/* Author: Constantinos Chamzas */

// Robowflex
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_scenes_visualize.cpp
 * Visualizes a set of scenes and motion plans from requests in these scenes. A
 * number of example scene and planning request pairs are included in
 * 'package://robowflex_library/yaml/fetch_scenes'. See the associated
 * `fetch_scenes_benchmark.cpp` for benchmarking over these scenes. See
 * https://kavrakilab.github.io/robowflex/rviz.html for RViz visualization.
 */

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();

    // False does not add the virtual joint since the real robot does not have one.
    fetch->initialize(false);

    // Create an RViz visualization helper.
    // Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(fetch);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    const int start = 1;
    const int end = 10;
    for (int i = start; i <= end; i++)
    {
        const auto &is = std::to_string(i);
        const auto &index = std::string(4 - is.size(), '0') + is;

        const auto &scene_file =
            "package://robowflex_library/yaml/fetch_scenes/scene_vicon" + index + ".yaml";
        const auto &request_file = "package://robowflex_library/yaml/fetch_scenes/request" + index + ".yaml";

        // Create an empty Scene.
        auto scene = std::make_shared<Scene>(fetch);
        if (not scene->fromYAMLFile(scene_file))
        {
            RBX_ERROR("Failed to read file: %s for scene", scene_file);
            continue;
        }

        // Create the default planner for the Fetch.
        auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
        planner->initialize();

        // Create an empty motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(planner, GROUP);
        if (not request->fromYAMLFile(request_file))
        {
            RBX_ERROR("Failed to read file: %s for request", request_file);
            continue;
        }

        // Visualize the scene.
        rviz.updateScene(scene);
        rviz.updateMarkers();

        RBX_INFO("Scene displayed! Press enter to plan...");
        std::cin.get();

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(scene, request->getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;

        // Publish the trajectory to a topic to display in RViz
        rviz.updateTrajectory(res);

        RBX_INFO("Press enter to remove the scene.");
        std::cin.get();

        rviz.removeScene();
    }
}
