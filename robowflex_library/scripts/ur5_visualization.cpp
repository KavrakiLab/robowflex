/* Author: Zachary Kingston */

#include <robowflex_library/robowflex.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    IO::RVIZHelper rviz(ur5);

    auto scene = std::make_shared<Scene>(ur5);

    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    MotionRequestBuilder request(planner, "manipulator");
    request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    auto sphere = std::make_shared<Geometry>(Geometry::ShapeType::SPHERE, Eigen::Vector3d{0.01, 0, 0});
    request.setGoalRegion("ee_link", "world",      // links
                          pose, sphere,            // position
                          orn, {0.01, 0.01, 0.01}  // orientation
    );

    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    rviz.updateTrajectory(res);

    ur5->dumpGeometry("ur5.yaml");
    ur5->dumpPathTransforms(*res.trajectory_, "ur5_path.yaml");

    while (ros::ok())
        ros::spinOnce();

    return 0;
}
