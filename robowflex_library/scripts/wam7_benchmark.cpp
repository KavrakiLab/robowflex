#include <ros/ros.h>
#include <signal.h>

#include <robowflex_library/robowflex.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    Robot wam7("wam7");
    wam7.initialize("package://barrett_model/robots/wam7_bhand.urdf.xacro",  // urdf
                    "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                    "package://barrett_wam_moveit_config/config/joint_limits.yaml",  // joint limits
                    "package://barrett_wam_moveit_config/config/kinematics.yaml"     // kinematics
    );

    Scene scene(wam7);

    OMPL::OMPLPipelinePlanner planner(wam7);
    planner.initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    MotionRequestBuilder request(planner, "arm");
    request.setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request.setGoalConfiguration({0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    wam7.loadKinematics("arm");

    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("test", scene, planner, request);

    BenchmarkOutputterPtr out_json(new JSONBenchmarkOutputter("test_log.json"));
    BenchmarkOutputterPtr out_traj(new TrajectoryOutputter("test_log.bag"));
    std::vector<BenchmarkOutputterPtr> outs;
    outs.push_back(out_json);
    outs.push_back(out_traj);
    benchmark.benchmark(outs);

    return 0;
}
