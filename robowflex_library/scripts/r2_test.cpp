#include <ros/ros.h>
#include <signal.h>

#include "robowflex.h"
#include "detail/r2.h"

using namespace robowflex;

void shutdown(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);

    R2Robot r2;
    r2.initialize({"legsandtorso"});

    Scene scene(r2);

    OMPL::R2OMPLPipelinePlanner planner(r2);
    planner.initialize();

    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan.yml");

    planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}
