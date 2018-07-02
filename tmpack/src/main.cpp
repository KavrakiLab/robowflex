#include "footstep_planner/my_walker.cpp"
#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/r2.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>

#define NUM_ITERATIONS 100
// #define START_POSE                                                                                                     \
//     {                                                                                                                  \
//         -0.030580680548965233, 0.05347800856425433, 0.015236617202923242, 1.6048735607571416, -0.08929297054119978,    \
//             1.633835488060198, 1.6268710455959683, 0.5000000000000027, 0.4999999999999982, 0.5, -0.026793193509068836, \
//             -0.0601802322235212, 0.025380777360997087, 1.5383038467290353, -0.08692033328500415, 1.507065006616994,    \
//             1.6211921121057182, 0.0, 0.0, 0.0, 1.5987211554602254e-14, 0.8726646259971691, -1.3962634015954636,        \
//             -1.8325957145940468, -2.4434609527920617, 1.3962634015954638, 0.0, -8.881784197001252e-16,                 \
//             1.7763568394002505e-15, 8.881784197001252e-16, 8.881784197001252e-16, 0.0, 8.881784197001252e-16,          \
//             1.7763568394002505e-15, 8.881784197001252e-16, 8.881784197001252e-16, 8.881784197001252e-16,               \
//             8.881784197001252e-16, 0.0, 8.881784197001252e-16, 0.0, 8.881784197001252e-16, 0.0, 0.0,                   \
//             8.881784197001252e-16, 0.0, -0.8726646259972393, -1.3962634015953872, 1.8325957145941594,                  \
//             -2.4434609527919715, -1.396263401596698, -2.4424906541753444e-13, 5.782041512247815e-13,                   \
//             1.865174681370263e-14, 1.7763568394002505e-15, 2.6645352591003757e-15, 0.0, 1.7763568394002505e-14,        \
//             -3.552713678800501e-15, -3.552713678800501e-15, -8.881784197001252e-16, -1.0658141036401503e-14,           \
//             -7.993605777301127e-15, -8.881784197001252e-16, -8.881784197001252e-15, -7.105427357601002e-15, 0.0,       \
//             3.019806626980426e-14, 1.2434497875801753e-14, 3.019806626980426e-14, 8.881784197001252e-16,               \
//             -0.0872664625997146, -1.7763568394002505e-15, 8.881784197001252e-16                                        \
//     }

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

    //R2RobotPtr r2 = std::make_shared<R2Robot>();
    auto r2 = std::make_shared<R2Robot>();

    r2->initialize({"legsandtorso"});
    //ScenePtr scene = std::make_shared<Scene>(r2);
    auto scene = std::make_shared<Scene>(r2);

    //OMPL::R2OMPLPipelinePlannerPtr planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);

    planner->initialize();
    auto request = std::make_shared<MotionRequestBuilder>(planner, "legsandtorso");
    request->fromYAMLFile("package://robowflex_library/yaml/r2_start.yml");

    size_t time_spent = 0;
    size_t count = 0;
    size_t success_count = 0;
    ros::Rate rate(0.5);
    // std::vector<double> start = START_POSE; //We ignore this for the YAML start

    MyWalker walker(r2, "legsandtorso", planner, scene, request);

    //For r2_start.yml
    std::vector<double> tmp = {1.98552, 0.0242871, 9.14127e-05, 4.8366e-06, -2.4964e-06, 1, -6.53607e-07};
    std::vector<double> start_joint_positions = request->getRequest().start_state.joint_state.position;
    tmp.insert(tmp.end(), start_joint_positions.begin(), start_joint_positions.end());
    start_joint_positions = tmp;

    robowflex::IO::RVIZHelper rviz;
    //rviz.updateScene(scene);
    //int a;
    //std::cin >> a;
    for (; count < NUM_ITERATIONS; count++)
    {
        size_t begin = ros::Time::now().nsec;

        //reset start position before each iteration
        r2->setState(start_joint_positions);
        std::vector<planning_interface::MotionPlanResponse> res = walker.plan();
        // planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
        if (res.back().error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            //rviz.updateTrajectory(res[0]);
            success_count++;
        }
        time_spent += (ros::Time::now().nsec - begin);
        //std::cin >> a;
    }

    std::cout << "Time spent: " << time_spent << std::endl;
    std::cout << "Number of runs: " << count << std::endl;
    std::cout << "Number of successful runs: " << success_count << std::endl;
    return 0;
}
