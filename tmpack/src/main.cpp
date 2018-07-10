#include "footstep_planner/my_walker.cpp"
#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/r2.h>
#include <robowflex_library/io/visualization.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>

#define NUM_ITERATIONS 1

using namespace robowflex;

void shutdown(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robowflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);

    // R2RobotPtr r2 = std::make_shared<R2Robot>();
    auto r2 = std::make_shared<R2Robot>();

    r2->initialize({"legsandtorso"});
    // ScenePtr scene = std::make_shared<Scene>(r2);
    auto scene = std::make_shared<Scene>(r2);

    // OMPL::R2OMPLPipelinePlannerPtr planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);

    planner->initialize();
    auto request = std::make_shared<MotionRequestBuilder>(planner, "legsandtorso");
    request->fromYAMLFile("package://robowflex_library/yaml/r2_start.yml");

    size_t time_spent = 0;
    size_t count = 0;
    size_t success_count = 0;
    ros::Rate rate(0.5);
    // std::vector<double> start = START_POSE; //We ignore this for the YAML start

    // For r2_start.yml
    std::vector<double> tmp = {1.98552, 0.0242871, 9.14127e-05, 4.8366e-06, -2.4964e-06, 1, -6.53607e-07};
    std::vector<double> start_joint_positions = request->getRequest().start_state.joint_state.position;
    tmp.insert(tmp.end(), start_joint_positions.begin(), start_joint_positions.end());
    start_joint_positions = tmp;

    IO::RVIZHelper rviz = IO::RVIZHelper(r2, "robonaut2");

    MyWalker walker(r2, "legsandtorso", planner, scene, request, rviz);

    int a;
    std::cin >> a;
    for (; count < NUM_ITERATIONS; count++)
    {
        size_t begin = ros::Time::now().nsec;

        // reset start position before each iteration
        r2->setState(start_joint_positions);
        std::vector<planning_interface::MotionPlanResponse> res = walker.plan();
        // planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
        if (res.back().error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            success_count++;
        }
        else
        {
            request->toYAMLFile("/home/awells/failed_motions/" + std::to_string(count) + ".yml");
        }

        rviz.updateTrajectories(res);
        // rviz.updateMarkers();

        time_spent += (ros::Time::now().nsec - begin);
    }

    while (true)
        ros::spinOnce();

    std::cout << "Time spent: " << time_spent << std::endl;
    std::cout << "Number of runs: " << count << std::endl;
    std::cout << "Number of successful runs: " << success_count << std::endl;

    return 0;
}
