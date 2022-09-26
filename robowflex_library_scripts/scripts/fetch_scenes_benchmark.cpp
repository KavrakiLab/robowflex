/* Author: Constantinos Chamzas */

// Robowflex
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_scenes_benchmark.cpp
 * A script that demonstrates benchmarking over a number of scenes and planning
 * requests for the Fetch robot. A number of example scene and planning request
 * pairs are included in 'package://robowflex_library/yaml/fetch_scenes'. This
 * script sets up benchmarking for all of these pairs. See
 * `fetch_scenes_visualize.cpp` to visualize these scenes.
 *
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 */

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();

    // Do not add the virtual joint since the real robot does not have one.
    fetch->initialize(false);

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH;
    Experiment experiment("fetch_scenes", options, 10.0, 10);

    const std::size_t start = 1;
    const std::size_t end = 10;
    for (std::size_t i = start; i <= end; i++)
    {
        const auto &scene_file =
            log::format("package://robowflex_library/yaml/fetch_scenes/scene_vicon%1$04d.yaml", i);
        const auto &request_file =
            log::format("package://robowflex_library/yaml/fetch_scenes/request%1$04d.yaml", i);

        // Create an empty Scene.
        auto scene = std::make_shared<Scene>(fetch);
        if (not scene->fromYAMLFile(scene_file))
        {
            RBX_ERROR("Failed to read file: %s for scene", scene_file);
            continue;
        }

        // Create the default planner for the Fetch.
        auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch);

        // Disable simplification
        auto settings = OMPL::Settings();
        settings.simplify_solutions = false;

        planner->initialize(settings);

        // Create an empty motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(planner, GROUP);
        if (not request->fromYAMLFile(request_file))
        {
            RBX_ERROR("Failed to read file: %s for request", request_file);
            continue;
        }

        // Add request
        experiment.addQuery("vicon", scene, planner, request);
    }

    auto dataset = experiment.benchmark(4);

    OMPLPlanDataSetOutputter output("robowflex");
    output.dump(*dataset);

    return 0;
}
