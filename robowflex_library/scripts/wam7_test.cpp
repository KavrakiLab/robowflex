#include <robowflex_library/robowflex.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    Robot wam7("wam7");
    wam7.initialize("package://barrett_model/robots/wam_7dof_wam_bhand.urdf.xacro",  // urdf
                    "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                    "package://barrett_wam_moveit_config/config/joint_limits.yaml",  // joint limits
                    "package://barrett_wam_moveit_config/config/kinematics.yaml"     // kinematics
    );

    Scene scene(wam7);

    OMPL::OMPLPipelinePlanner planner(wam7);
    OMPL::Settings settings;
    settings.simplify_solutions = false;
    planner.initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml",  // planner config
                       settings
    );

    MotionRequestBuilder request(planner, "arm");
    request.setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request.setGoalConfiguration({0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0});
    request.setConfig("BKPIECE");

    wam7.loadKinematics("arm");

    planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
