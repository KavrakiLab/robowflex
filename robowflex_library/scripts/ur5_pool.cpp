/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file ur5_pool.cpp
 * Demonstration of PoolPlanner with the UR5. Here, many planning requests are
 * executed asynchronously.
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

    // Create a pool of default planners for the UR5.
    auto planner = std::make_shared<PoolPlanner>(ur5);
    planner->initialize<OMPL::UR5OMPLPipelinePlanner>();

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

    // Submit a blocking call for planning. This plan is executed in a different thread
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Submit a set of asynchronous planning calls.
    auto job1 = planner->submit(scene, request.getRequest());
    auto job2 = planner->submit(scene, request.getRequest());
    auto job3 = planner->submit(scene, request.getRequest());
    auto job4 = planner->submit(scene, request.getRequest());
    auto job5 = planner->submit(scene, request.getRequest());
    auto job6 = planner->submit(scene, request.getRequest());
    auto job7 = planner->submit(scene, request.getRequest());
    auto job8 = planner->submit(scene, request.getRequest());

    // Cancel a job (if already running, nothing happens, but if canceled before execution the job is skipped)
    job5->cancel();

    // Wait for a job to complete
    job1->wait();

    // Get results of plans. These block until results are available.
    planning_interface::MotionPlanResponse res1 = job1->get();
    planning_interface::MotionPlanResponse res2 = job2->get();
    planning_interface::MotionPlanResponse res3 = job3->get();
    planning_interface::MotionPlanResponse res4 = job4->get();

    // We canceled job5, so no result is guaranteed.
    // planning_interface::MotionPlanResponse res5 = job5->get();

    planning_interface::MotionPlanResponse res6 = job6->get();
    planning_interface::MotionPlanResponse res7 = job7->get();
    planning_interface::MotionPlanResponse res8 = job8->get();

    return 0;
}
