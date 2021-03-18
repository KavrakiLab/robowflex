/* Author: Constantinos Chamzas, Zachary Kingston */

#include <robowflex_library/log.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();

    // False does not add the virtual joint since the real robot does not have one.
    fetch->initialize(false);

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Benchmarker benchmark;

    int start = 1;
    int end = 10;
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
            RBX_ERROR("Failed to read file: %s for scene", scene_file.c_str());
            continue;
        }

        // Create the default planner for the Fetch.
        auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch, "default");
        planner->initialize("package://fetch_moveit_config/config/ompl_planning.yaml");

        // Create an empty motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(planner, GROUP);
        if (not request->fromYAMLFile(request_file))
        {
            RBX_ERROR("Failed to read file: %s for request", request_file.c_str());
            continue;
        }

        // Modify the planning request here
        request->setNumPlanningAttempts(1);
        request->setAllowedPlanningTime(200);
        request->setConfig("PRMstar");

        // Add benchmarking request
        benchmark.addBenchmarkingRequest("result" + index, scene, planner, request);
    }

    // How many times to solve each problem
    unsigned int reps = 2;

    Benchmarker::Options options(reps, Benchmarker::WAYPOINTS | Benchmarker::CORRECT | Benchmarker::LENGTH);
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_fetch_ompl_scene_benchmark/")},
                        options);
}
