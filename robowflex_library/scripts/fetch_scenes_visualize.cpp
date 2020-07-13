/* Author: Constantinos Chamzas */

// Robowflex
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

    // Create an RViz visualization helper.
    // Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(fetch);

    ROS_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    int start = 1;
    int end = 10;
    for (int i = start; i <= end; i++)
    {
        std::stringstream ss;
        // (pre-appending 4 leading zeros)
        ss << std::setw(4) << std::setfill('0') << i;
        std::string index = ss.str();

        auto fscene = "package://robowflex_library/yaml/fetch_scenes/scene_vicon" + index + ".yaml";

        // Create an empty Scene.
        auto scene = std::make_shared<Scene>(fetch);

        if (!scene->fromYAMLFile(fscene))
        {
            ROS_ERROR("Failed to read file: %s for scene", fscene.c_str());
            continue;
        }

        // Create the default planner for the Fetch.
        auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
        planner->initialize();

        // Create an empty motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(planner, GROUP);

        auto frequest = "package://robowflex_library/yaml/fetch_scenes/request" + index + ".yaml";

        if (!request->fromYAMLFile(frequest))
        {
            ROS_ERROR("Failed to read file: %s for request", frequest.c_str());
            continue;
        }

        // Visualize the scene.
        rviz.updateScene(scene);
        rviz.updateMarkers();

        ROS_INFO("Scene displayed! Press enter to plan...");
        std::cin.get();

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(scene, request->getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;

        // Publish the trajectory to a topic to display in RViz
        rviz.updateTrajectory(res);

        ROS_INFO("Press enter to remove the scene.");
        std::cin.get();

        rviz.removeScene();
    }
}
