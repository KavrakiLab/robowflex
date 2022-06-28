/* Author: Carlos Quintero-Peña */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/stretch.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_library/random.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>

using namespace robowflex;

/* \file stretch_test.cpp
 * A simple script that demonstrates motion planning with the Stretch robot. The
 * resulting trajectory is output to a YAML file. This file can be visualized
 * using Blender. See the corresponding robowflex_visualization readme.
 */

static const std::string GROUP = "stretch_arm";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Stretch robot.
    auto stretch = std::make_shared<StretchRobot>();
    stretch->initialize();

    // Open Gripper
    stretch->openGripper();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(stretch);
    scene->toYAMLFile("ex_stretch.yml");

    // Create the default planner for the Stretch.
    auto planner = std::make_shared<OMPL::StretchOMPLPipelinePlanner>(stretch, "default");
    planner->initialize();

    // Sets the Stretch's head pose to look at a point.
    stretch->pointHead({0, -2, 10});

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, GROUP);
    stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0});  // home
    request.setStartConfiguration(stretch->getScratchState());

    stretch->setGroupState(GROUP, {0.9967, 0.0, 0.13, 0.13, 0.13, 0.13, 4});  // extended
    request.setGoalConfiguration(stretch->getScratchState());

    request.setConfig("RRTConnect");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("stretch_path.yml");

    return 0;
}
