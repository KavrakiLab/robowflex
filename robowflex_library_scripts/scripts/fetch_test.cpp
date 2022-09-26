/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_test.cpp
 * A simple script that demonstrates motion planning with the Fetch robot. The
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

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Sets the Fetch's base pose.
    fetch->setBasePose(1, 1, 0.5);

    // Sets the Fetch's head pose to look at a point.
    fetch->pointHead({2, 1, 1.5});

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, GROUP);
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request.setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request.setGoalConfiguration(fetch->getScratchState());

    request.setConfig("RRTConnect");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("fetch_path.yml");

    return 0;
}
