#include "footstep_planner/my_walker.cpp"
#include "robowflex.h"
#include <ros/ros.h>
#include <signal.h>

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

    OMPL::OMPLPipelinePlanner planner(ur5);
    planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    // OMPL::OMPLInterfacePlanner planner(ur5);
    // planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    // );

    // Geometry box(Geometry::ShapeType::BOX, {0.1, 0.1, 0.1});
    // Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    // pose.translate(Eigen::Vector3d{1, 1, 1});

    // scene.addCollisionObject("box", box, pose);

    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});
    request.setGoalConfiguration({-0.39, -0.69, -2.12, 2.82, -0.39, 0.0});

    std::vector<double> start = {0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0};
    MyWalker walker(ur5, "manipulator", planner, scene, request, start);

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        auto res = walker.plan();

        // rviz.update(res);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
