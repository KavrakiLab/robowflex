#include "footstep_planner/my_walker.cpp"
#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/r2.h>
#include <ros/ros.h>
#include <signal.h>

#define NUM_ITERATIONS 1
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

    R2Robot r2;
    r2.initialize({"legsandtorso"});
    Scene scene(r2);
    OMPL::R2OMPLPipelinePlanner planner(r2);
    planner.initialize();
    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_start.yml");

    size_t time_spent = 0;
    size_t count = 0;
    size_t success_count = 0;
    ros::Rate rate(0.5);
    // std::vector<double> start = START_POSE; //We ignore this for the YAML start

    MyWalker walker(r2, "legsandtorso", planner, scene, request);
    while (count++ < NUM_ITERATIONS)
    {
        size_t begin = ros::Time::now().nsec;
        std::vector<planning_interface::MotionPlanResponse> res = walker.plan();
        // planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
        if (res[0].error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            success_count++;
        }
        time_spent += (ros::Time::now().nsec - begin);
    }

    std::cout << "Time spent: " << time_spent << std::endl;
    std::cout << "Number of runs: " << count << std::endl;
    std::cout << "Number of successful runs: " << success_count << std::endl;
    return 0;
}
