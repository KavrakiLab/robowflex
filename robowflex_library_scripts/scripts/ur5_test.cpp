/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_test.cpp
 * A simple script that demonstrates motion planning with the UR5 robot. Here,
 * two planners are created: the default OMPL planner, and the default OMPL
 * planner but with simplified solutions disabled.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);

    // Create the default planner for the UR5.
    auto default_planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5, "default");
    default_planner->initialize();

    // Create the a planner for the UR5, and disable simplification.
    auto simple_planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    simple_planner->initialize(settings);

    // Run a motion plan for each planner.
    for (const auto &planner : {default_planner, simple_planner})
    {
        // Create a motion planning request with a pose goal.
        MotionRequestBuilder request(planner, "manipulator");
        request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

        RobotPose pose = RobotPose::Identity();
        pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
        Eigen::Quaterniond orn{0, 0, 1, 0};

        request.setGoalRegion("ee_link", "world",               // links
                              pose, Geometry::makeSphere(0.1),  // position
                              orn, {0.01, 0.01, 0.01}           // orientation
        );

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;
    }

    return 0;
}
