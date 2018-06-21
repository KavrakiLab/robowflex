#include <robowflex_library/robowflex.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    auto wam7 = std::make_shared<Robot>("wam7");
    wam7->initialize("package://barrett_model/robots/wam7_bhand.urdf.xacro",          // urdf
                     "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                     "package://barrett_wam_moveit_config/config/joint_limits.yaml",  // joint limits
                     "package://barrett_wam_moveit_config/config/kinematics.yaml"     // kinematics
    );

    auto scene = std::make_shared<Scene>(wam7);

    auto planner = std::make_shared<OMPL::OMPLPipelinePlanner>(wam7);
    planner->initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, "arm"));
    request->setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request->setGoalConfiguration({0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    wam7->loadKinematics("arm");

    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("test", scene, planner, request);

    Benchmarker::Options options;
    options.run_metric_bits = Benchmarker::RunMetricBits::PATH |
                              Benchmarker::RunMetricBits::LENGTH;
    options.runs = 10;
    benchmark.benchmark({std::make_shared<JSONBenchmarkOutputter>("test_log.json"),  //
                         std::make_shared<TrajectoryBenchmarkOutputter>("test_log.bag")},
                         options);
    return 0;
}
