#include <ros/ros.h>
#include <signal.h>

#include <robowflex_library/robowflex.h>

using namespace robowflex;

void shutdown(int sig)
{
    ros::spinOnce();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);

    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);

    Robot wam7("wam7");
    wam7.initialize("package://barrett_model/robots/wam_7dof_wam_bhand.urdf.xacro",  // urdf
                   "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                   "package://barrett_wam_moveit_config/config/joint_limits.yaml",      // joint limits
                   "package://barrett_wam_moveit_config/config/kinematics.yaml"         // kinematics
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

    JSONBenchmarkOutputter out("test_log.json");
    benchmark.benchmark(out);

    return 0;
}
