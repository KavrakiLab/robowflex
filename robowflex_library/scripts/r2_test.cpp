#include <ros/ros.h>
#include <signal.h>

#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/r2.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    R2Robot r2;
    r2.initialize({"legsandtorso"});

    ScenePtr scene(new Scene(r2));

    OMPL::R2OMPLPipelinePlannerPtr planner(new OMPL::R2OMPLPipelinePlanner(r2));
    planner->initialize();

    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan.yml");

    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
