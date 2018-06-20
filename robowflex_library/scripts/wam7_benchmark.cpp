#include <robowflex_library/robowflex.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    Robot wam7("wam7");
    wam7.initialize("package://barrett_model/robots/wam7_bhand.urdf.xacro",          // urdf
                    "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                    "package://barrett_wam_moveit_config/config/joint_limits.yaml",      // joint limits
                    "package://barrett_wam_moveit_config/config/kinematics.yaml"         // kinematics
    );

    Scene scene(wam7);
    scene.fromOpenRAVEXMLFile("package://OptPlanners_OpenRAVE/scripts/data/envs/wam7_table_andrewshelf.env.xml");

    OMPL::OMPLPipelinePlanner planner(wam7);
    planner.initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    MotionRequestBuilder request(planner, "arm");
    request.setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request.setGoalConfiguration({0.0, 1.89, 0.0, -0.3, 1.3, 0.0, 0.2});

    wam7.loadKinematics("arm");

    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("test", scene, planner, request);

    Benchmarker::Options opts;
    opts.runs = 25;

    benchmark.benchmark({std::make_shared<JSONBenchmarkOutputter>("test_log.json"),  //
                         std::make_shared<TrajectoryBenchmarkOutputter>("test_log.bag")}, opts);
    return 0;
}
