/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/detail/fetch.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    startROS(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Dump the geometry information for visualization.
    fetch->dumpGeometry("fetch.yml");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "arm_with_torso");
    request.setStartConfiguration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    request.setGoalConfiguration({0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Output transforms from path to a file for visualization.
    fetch->dumpPathTransforms(*res.trajectory_, "fetch_path.yml");

    return 0;
}
