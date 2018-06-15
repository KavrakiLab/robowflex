#include <ros/ros.h>
#include <signal.h>

#include "robowflex.h"

using namespace robowflex;

void shutdown(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);

    Robot r2("r2");
    r2.initialize("package://r2_description/urdf/r2c6.urdf",              // urdf
                  "package://r2_moveit_config/config/r2.srdf",            // srdf
                  "package://r2_moveit_config/config/joint_limits.yaml",  // joint limits
                  "package://r2_moveit_config/config/kinematics.yaml"     // kinematics
                  );

    r2.loadXMLFile("legs/simplified_robot_description", "package://r2_simplified_urdf/r2c6_legs_only_creepy.xacro");

    // These need to go in the node namespace
    ros::NodeHandle nh("~");
    nh.setParam("cached_ik_path", IO::resolvePath("package://robot_ragdoll_demos/config"));
    nh.setParam("constraint_samplers", "moveit_r2_constraints/MoveItR2ConstraintSamplerAllocator "
                                       "moveit_r2_constraints/MoveItR2PoseSamplerAllocator "
                                       "moveit_r2_constraints/MoveItR2JointConstraintSamplerAllocator");

    r2.loadKinematics("legsandtorso");

    Scene scene(r2);

    OMPL::OMPLPipelinePlanner planner(r2);
    planner.initialize("package://r2_moveit_config/config/ompl_planning.yaml",  // planner config
                       OMPL::Settings(),                                        // settings
                       "ompl_interface/OMPLPlanningContextManager"              // plugin
                       );

    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan.yml");

    planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
