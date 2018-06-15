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

    OMPL::OMPLPipelinePlanner planner(ur5);
    planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    // OMPL::OMPLInterfacePlanner planner(ur5);
    // planner.initialize("package://ur5_robotiq85_moveit_config/config/ompl_planning.yaml"  // planner config
    // );

    // Geometry box(Geometry::ShapeType::BOX, {0.1, 0.1, 0.1});
    // Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    // pose.translate(Eigen::Vector3d{1, 1, 1});

    // scene.updateCollisionObject("box", box, pose);

    // Geometry mesh(Geometry::ShapeType::MESH, {1, 1, 1}, "package://ur_description/meshes/sensors/kinect.dae");
    // pose.translate(Eigen::Vector3d{-2, 0, 0});

    // scene.updateCollisionObject("mesh", mesh, pose);

    // scene.fromYAMLFile("package://robowflex_library/yaml/test.yml");
    // scene.toYAMLFile("scene.yml");

    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    request.setGoalRegion("ee_link", "world",                                         // links
                          pose, Geometry(Geometry::ShapeType::SPHERE, {0.01, 0, 0}),  // position
                          orn, {0.01, 0.01, 0.01}                                     // orientation
    );

    ur5.loadKinematics("manipulator");

    RVIZHelper rviz(ur5, scene);

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            break;

        rviz.update(res);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
