/* Author: Constantinos Chamzas */

// Robowflex
#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/detail/fetch.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);  // false does not add the virtual joint.

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Benchmarker benchmark;

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
        // modify the planning request here
        request->setNumPlanningAttempts(1);
        request->setAllowedPlanningTime(10);

        // Add benchmarking request
        benchmark.addBenchmarkingRequest("result" + index, scene, planner, request);
    }

    // How many times to solve each problem
    int reps = 2;
    // 0b001101 disables costly metrics(some crash if no plan is found)
    auto options = Benchmarker::Options(reps, 0b001101);
    std::string bpath = ros::package::getPath("robowflex_library") + "/yaml/benchmark/";
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>(bpath)}, options);
}
