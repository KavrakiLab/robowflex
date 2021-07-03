/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_chomp.cpp
 * A basic script that demonstrates using MoveIt's built-in CHOMP planner. The
 * resulting trajectory is output to a YAML file. This file can be visualized
 * using Blender. See the corresponding robowflex_visualization readme.
 */

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);

    // Create a CHOMP planner for Fetch.
    auto planner = std::make_shared<opt::CHOMPPipelinePlanner>(fetch);
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, GROUP);
    fetch->setGroupState(GROUP, {0.265, 0.201, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});
    request.setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 1.301, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});
    request.setGoalConfiguration(fetch->getScratchState());

    request.setConfig("RRTConnect");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("fetch_chomp_path.yml");

    return 0;
}
