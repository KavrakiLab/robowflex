/* Author: Carlos Quintero Pena*/

#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

/* \file fetch_tabletop_planning_time.cpp
 A simple script that shows how to use TrajOpt with a planning time budget. The problem and scenes are loaded
 from yaml files. An RVIZ Helper object is used to visualize the start/goal states and the computed
 trajectory.
 */

static const std::string GROUP = "arm";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);

    // Load tabletop scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_tesseract/scenes/table/scene.yaml");

    // Attach object to end effector.
    scene->attachObject(*fetch->getScratchState(), "Can1");

    // Create a TrajOpt planner for Fetch.
    auto planner = std::make_shared<TrajOptPlanner>(fetch, GROUP);
    planner->initialize("torso_lift_link", "gripper_link");

    // Set planner parameters.
    planner->options.num_waypoints = 8;  // Select number of waypoints in trajectory.
    planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);  // Initialize using a straight-line
                                                                        // between start and goal in C-Space.

    // Load request.
    const auto &request = std::make_shared<MotionRequestBuilder>(fetch);
    request->fromYAMLFile("package://robowflex_tesseract/scenes/table/request.yaml");

    // RVIZ helper.
    const auto &rviz = std::make_shared<IO::RVIZHelper>(fetch);
    rviz->updateScene(scene);
    rviz->visualizeState(request->getStartConfiguration());

    RBX_INFO("Visualizing start state");
    RBX_INFO("Press Enter to run the planner and returning its first solution");
    std::cin.ignore();

    // Run the planner just once.
    planner->options.return_first_sol = true;

    // Do motion planning.
    auto res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        rviz->updateTrajectory(res);

    rviz->visualizeState(request->getGoalConfiguration());

    RBX_INFO("Visualizing goal state");
    RBX_INFO("Press Enter to run the planner with a time bound but returning as soon as it finds the first "
             "feasible solution");
    std::cin.ignore();

    // Run the planner potentially more than once and make it return when it finds a feasible solution or when
    // it runs out of time. Each time the planner runs, it starts with a perturbed version of the initial
    // trajectory.
    planner->options.return_first_sol = false;
    planner->options.return_after_timeout = false;

    // The time budget is taken from the planning request.
    request->getRequest().allowed_planning_time = 3.0;

    // Do motion planning.
    res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        rviz->updateTrajectory(res);

    rviz->visualizeState(request->getGoalConfiguration());

    RBX_INFO("Visualizing goal state");
    RBX_INFO("Press Enter to run the planner for the whole time budget");
    std::cin.ignore();

    // Run the planner for the whole time budget.
    planner->options.return_first_sol = false;
    planner->options.return_after_timeout = true;

    // The time budget is taken from the planning request.
    request->getRequest().allowed_planning_time = 3.0;

    // Do motion planning.
    res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        rviz->updateTrajectory(res);

    rviz->visualizeState(request->getGoalConfiguration());

    RBX_INFO("Visualizing goal state");
    RBX_INFO("Press Enter to finish");
    std::cin.ignore();

    return 0;
}
