#include "footstep_planner/my_walker.cpp"
#include <include/robowflex.h>
#include <ros/ros.h>
#include <signal.h>

#define NUM_ITERATIONS 1
#define START_POSE                                                                                                     \
    {                                                                                                                  \
        -0.030580680548965233, 0.05347800856425433, 0.015236617202923242, 1.6048735607571416, -0.08929297054119978,    \
            1.633835488060198, 1.6268710455959683, 0.5000000000000027, 0.4999999999999982, 0.5, -0.026793193509068836, \
            -0.0601802322235212, 0.025380777360997087, 1.5383038467290353, -0.08692033328500415, 1.507065006616994,    \
            1.6211921121057182, 0.0, 0.0, 0.0, 1.5987211554602254e-14, 0.8726646259971691, -1.3962634015954636,        \
            -1.8325957145940468, -2.4434609527920617, 1.3962634015954638, 0.0, -8.881784197001252e-16,                 \
            1.7763568394002505e-15, 8.881784197001252e-16, 8.881784197001252e-16, 0.0, 8.881784197001252e-16,          \
            1.7763568394002505e-15, 8.881784197001252e-16, 8.881784197001252e-16, 8.881784197001252e-16,               \
            8.881784197001252e-16, 0.0, 8.881784197001252e-16, 0.0, 8.881784197001252e-16, 0.0, 0.0,                   \
            8.881784197001252e-16, 0.0, -0.8726646259972393, -1.3962634015953872, 1.8325957145941594,                  \
            -2.4434609527919715, -1.396263401596698, -2.4424906541753444e-13, 5.782041512247815e-13,                   \
            1.865174681370263e-14, 1.7763568394002505e-15, 2.6645352591003757e-15, 0.0, 1.7763568394002505e-14,        \
            -3.552713678800501e-15, -3.552713678800501e-15, -8.881784197001252e-16, -1.0658141036401503e-14,           \
            -7.993605777301127e-15, -8.881784197001252e-16, -8.881784197001252e-15, -7.105427357601002e-15, 0.0,       \
            3.019806626980426e-14, 1.2434497875801753e-14, 3.019806626980426e-14, 8.881784197001252e-16,               \
            -0.0872664625997146, -1.7763568394002505e-15, 8.881784197001252e-16                                        \
    }

using namespace robowflex;

void shutdown(int sig)
{
    ros::spinOnce();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);

    Robot r2("r2");

    r2.initialize("package://r2_description/urdf/r2c6.urdf",              // urdf
                  "package://r2_moveit_config/config/r2.srdf",            // srdf
                  "package://r2_moveit_config/config/joint_limits.yaml",  // joint limits
                  "package://r2_moveit_config/config/kinematics.yaml"     // kinematics
    );

    Scene scene(r2);

    // skeleton of idea for passing constraints and scene graph changes through the tmp interface
    // add the handrails
    // scene.updateCollisionObject("ISS Handrail 0", ,);

    OMPL::OMPLPipelinePlanner planner(r2);
    planner.initialize("package://r2_moveit_config/config/ompl_planning.yaml",  // planner config
                       OMPL::Settings(),                                        // settings
                       "ompl_interface/OMPLPlanningContextManager"              // plugin
    );

    MotionRequestBuilder request(planner, "legsandtorso");
    // Lock the right foot in place:
    request.addPathPositionConstraint(
        "r2/right_leg/gripper/tip", "/world", Eigen::Affine3d(Eigen::Translation3d(1.983001, 0.321150, -1.361)),
        Geometry(Geometry::ShapeType::Type::SPHERE, Eigen::Vector3d(0.01, 0.01, 0.01), "right_foot_base_position"));

    std::vector<double> start = START_POSE;

    MyWalker walker(r2, "legsandtorso", planner, scene, request, start);

    size_t time_spent = 0;
    size_t count = 0;
    size_t success_count = 0;
    ros::Rate rate(0.5);

    while (ros::ok() && count++ < NUM_ITERATIONS)
    {
        size_t begin = ros::Time::now().nsec;
        auto res = walker.plan();
        if (res[0].error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            success_count++;

        time_spent += (ros::Time::now().nsec - begin);

        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Time spent: " << time_spent << std::endl;

    return 0;
}
