/** \author Bryce Willey */

#include <robowflex_library/builder.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create a WAM7 robot, specifying all necessary files.
    auto wam7 = std::make_shared<Robot>("wam7");
    wam7->initialize("package://barrett_model/robots/wam_7dof_wam_bhand.urdf.xacro",  // urdf
                     "package://barrett_wam_moveit_config/config/wam7_hand.srdf",     // srdf
                     "package://barrett_wam_moveit_config/config/joint_limits.yaml",  // joint limits
                     "package://barrett_wam_moveit_config/config/kinematics.yaml"     // kinematics
    );

    // Load kinematics for the WAM7 arm.
    wam7->loadKinematics("arm");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(wam7);

    // Create the default OMPL planner with path simplification disabled, with the WAM7 planning
    // configuration.
    auto planner = std::make_shared<OMPL::OMPLPipelinePlanner>(wam7);

    OMPL::Settings settings;
    settings.simplify_solutions = false;
    planner->initialize("package://barrett_wam_moveit_config/config/ompl_planning.yaml",  // planner config
                        settings);

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilder request(planner, "arm");
    request.setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request.setGoalConfiguration({0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0});
    request.setConfig("BKPIECE");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
