#include <boost/format.hpp>

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/detail/r2.h>

using namespace robowflex;

int planFromFile()
{
    // Create an R2 robot, initialize the `legsandtorso` kinematics solver.
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({"legsandtorso"});

    // Load the ISS from a world file.
    auto iss_scene = std::make_shared<Scene>(r2);
    iss_scene->fromYAMLFile("package://robowflex_library/yaml/bench/world.yml");

    // Create the default motion planner for R2.
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    planner->initialize();

    Benchmarker benchmark;

    for (unsigned int i = 1; i <= 6; ++i)
    {
        // Load a motion planning request (a step with a torso constraint).
        auto request1 = std::make_shared<MotionRequestBuilder>(planner, "legsandtorso");
        request1->fromYAMLFile(
            boost::str(boost::format{"package://robowflex_library/yaml/bench/r2_%1%.yml"} % i));
        request1->setConfig("RRTConnect_Constrained");
        request1->setAllowedPlanningTime(60);
        request1->getRequest().num_planning_attempts = 1;

        benchmark.addBenchmarkingRequest(boost::str(boost::format{"RRTConnect Step %1%"} % i), iss_scene,
                                         planner, request1);

        auto request2 = std::make_shared<MotionRequestBuilder>(planner, "legsandtorso");
        request2->fromYAMLFile(
            boost::str(boost::format{"package://robowflex_library/yaml/bench/r2_%1%.yml"} % i));
        request2->setConfig("CBiRRT2");
        request2->setAllowedPlanningTime(60);
        request2->getRequest().num_planning_attempts = 1;

        benchmark.addBenchmarkingRequest(boost::str(boost::format{"CBiRRT2 Step %1%"} % i), iss_scene,
                                         planner, request2);
    }

    // Set benchmarking options to only compute and store paths and path length.
    Benchmarker::Options options(10, Benchmarker::MetricOptions::PATH | Benchmarker::MetricOptions::LENGTH);

    // Benchmark and output to JSON and a rosbag of trajectories.
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_r2_test/")}, options);
    return 0;
}

int main(int argc, char **argv)
{
    // Startup ROS.
    ROS ros(argc, argv);

    // Plan using configuration from files.
    planFromFile();
}
