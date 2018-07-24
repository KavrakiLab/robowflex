#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_tesseract/tesseract_planners.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    startROS(argc, argv);

    // Create a UR5 robot, specifying all necessary files.
    auto ur5 = std::make_shared<Robot>("ur5");
    ur5->initialize("package://ur_description/urdf/ur5_joint_limited_robot.urdf.xacro",  // urdf
                    "package://ur5_moveit_config/config/ur5.srdf",                       // srdf
                    "package://ur5_moveit_config/config/joint_limits.yaml",              // joint limits
                    "package://ur5_moveit_config/config/kinematics.yaml"                 // kinematics
    );

    // Load kinematics for the UR5 arm.
    ur5->loadKinematics("manipulator");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);

    // Create the default OMPL planner, with continuous collision checking.
    hypercube::Settings settings;
    settings.simplify_solutions = true;
    settings.use_continuous_validator = true;

    auto planner = std::make_shared<hypercube::OMPLChainPlanner>(ur5);
    planner->initialize("package://ur5_moveit_config/config/ompl_planning.yaml", settings);

    // Create a motion planning request with a joint position goal.
    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request.setGoalConfiguration({0.0, -1.0, 0.0, 0.0, 0.0, 0.0});

    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
