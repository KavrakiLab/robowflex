/* Author: Carlos Quintero Pena*/

#include <robowflex_library/util.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

/* \file fetch_tabletop_goalpose.cpp
 * A simple script that shows how to use TrajOpt to plan in a manipulation task. The scene and request are
 * loaded from yaml files. Instead of providing a goal state, a goal pose for the end effector is encoded as a
 * cartesian constraint. An RVIZ Helper object is used to visualize the start/end states and the computed
 * trajectory.
 */

static const std::string GROUP = "arm";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);
    const auto &ee = fetch->getModel()->getEndEffectors()[0]->getLinkModelNames()[0];

    // Load tabletop scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_tesseract/scenes/table/scene.yaml");

    // Attach object to end effector.
    scene->attachObject(*fetch->getScratchState(), "Can1");
    fetch->getScratchState() =
        std::make_shared<robot_state::RobotState>(scene->getScene()->getCurrentStateNonConst());

    // Create a TrajOpt planner for Fetch.
    auto planner = std::make_shared<TrajOptPlanner>(fetch, GROUP);
    planner->initialize(GROUP + "_chain", "torso_lift_link", "gripper_link");
    planner->options.num_waypoints = 10;

    // Load request.
    const auto &request = std::make_shared<MotionRequestBuilder>(fetch);
    request->fromYAMLFile("package://robowflex_tesseract/scenes/table/request.yaml");
    const auto &start_state = request->getStartConfiguration();
    const auto &goal_state = request->getGoalConfiguration();
    const auto &goal_ee_pose = goal_state->getFrameTransform(ee);

    // RVIZ helper.
    const auto &rviz = std::make_shared<IO::RVIZHelper>(fetch);
    rviz->updateScene(scene);
    rviz->visualizeState(start_state);

    ROS_INFO("Visualizing start state");
    ROS_INFO("Press Enter to continue");
    std::cin.ignore();

    // Do motion planning.
    auto result = planner->plan(scene, start_state, goal_ee_pose, ee);
    if (result.first)
        rviz->updateTrajectory(planner->getTrajectory());

    rviz->visualizeState(planner->getTrajectory()->getLastWayPointPtr());

    ROS_INFO("Visualizing end state");
    ROS_INFO("Press Enter to exit");
    std::cin.ignore();

    return 0;
}
