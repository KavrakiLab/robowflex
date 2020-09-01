/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/path.h>

using namespace robowflex;

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
    // fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    fetch->setGroupState(GROUP, {0.265, 0.201, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request.setStartConfiguration(fetch->getScratchState());

    fetch->setGroupState(GROUP, {0.265, 0.701, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    request.setGoalConfiguration(fetch->getScratchState());

    request.setConfig("RRTConnect");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Output path to a file for visualization.
    path::toYAMLFile("fetch_chomp_path.yml", *res.trajectory_);
    return 0;
}
