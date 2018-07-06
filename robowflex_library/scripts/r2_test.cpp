#include <robowflex_library/util.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/detail/r2.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS.
    startROS(argc, argv);

    // Create an R2 robot, initialize the `legsandtorso` kinematics solver.
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({"legsandtorso"});

    // Dump the geometry information for visualization.
    r2->dumpGeometry("r2.yml");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(r2);

    // Create the default motion planner for R2.
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    planner->initialize();

    // Load a motion planning request (a step with a torso constraint).
    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan_waist.yml");

   // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Output transforms from path to a file for visualization.
    r2->dumpPathTransforms(*res.trajectory_, "r2_path.yml", 30, 0.5);

    return 0;
}
