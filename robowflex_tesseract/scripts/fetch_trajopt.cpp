/* Author: Carlos Quintero Pena */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

/* \file fetch_trajopt.cpp
 * A basic script that demonstrates using Tesseract's TrajOpt planner. The
 * resulting trajectory is output to a YAML file. This file can be visualized
 * using Blender. See the corresponding robowflex_visualization readme.
 */

static const std::string GROUP = "arm";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);

    // Create a TrajOpt planner for Fetch.
    auto planner = std::make_shared<TrajOptPlanner>(fetch, GROUP);
    planner->initialize("torso_lift_link", "gripper_link");
    planner->options.num_waypoints = 3;

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, GROUP);
    fetch->setGroupState(GROUP, {0.201, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});
    request.setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {1.301, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});
    request.setGoalConfiguration(fetch->getScratchState());

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("fetch_trajopt_path.yml");

    return 0;
}
