#include <ros/ros.h>
#include <signal.h>

#include "robowflex.h"

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

    Robot ur5("ur5");
    ur5.initialize("package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro",  // urdf
                   "package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf",     // srdf
                   "package://ur5_robotiq85_moveit_config/config/joint_limits.yaml",      // joint limits
                   "package://ur5_robotiq85_moveit_config/config/kinematics.yaml"         // kinematics
    );

    Scene scene(ur5);

    // OMPL::OMPLPipelinePlanner planner(ur5);
    // planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    // );

    OMPL::OMPLInterfacePlanner planner(ur5);
    // planner.initialize();
    planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    // Geometry box(Geometry::ShapeType::BOX, {0.1, 0.1, 0.1});
    // Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    // pose.translate(Eigen::Vector3d{1, 1, 1});

    // scene.addCollisionObject("box", box, pose);

    MotionRequestBuilder request(ur5, "manipulator");
    request.setStartConfiguration({   0.0,  -0.3,  -2.0,   0.0,   0.0,   0.0});
    request.setGoalConfiguration( { -0.39, -0.69, -2.12,  2.82, -0.39,   0.0});

    // RVIZHelper rviz(ur5, scene);

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            break;

        // rviz.update(res);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
