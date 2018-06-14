#include "footstep_planner/my_walker.cpp"
#include "../../robowflex_library/include/robowflex.h"
#include <ros/ros.h>
#include <signal.h>

#define NUM_ITERATIONS 1
#define START_POSE {1.98552, 0.0730371, 9.14127e-05, 4.8366e-06, -2.4964e-06, 1, -6.53607e-07, 0, 0, 0, 1.5708, 8.88178e-16, 1.5708, 1.5708, 0, 0, 0, 0, 0, -8.88178e-16, 1.5708, -8.88178e-16, 1.5708, 1.5708, 0, 0, 0, 0, 0.872665, -1.39626, -1.8326, -2.44346, 1.39626, 0, 0, 8.88178e-16, 0, -8.88178e-16, 8.88178e-16, 0, 8.88178e-16, -8.88178e-16, 0, 8.88178e-16, 0, 8.88178e-16, 8.88178e-16, 8.88178e-16, 0, 8.88178e-16, 0, 0, 0, -0.872665, -1.39626, 1.8326, -2.44346, -1.39626, 0, -8.88178e-16, 0, 0, 0, 0, 0, -8.88178e-16, 0, 0, 0, 8.88178e-16, 0, 0, 0, -8.88178e-16, 8.88178e-16, -8.88178e-16, -4.44089e-15, 1.77636e-15, -0.0872665, 1.77636e-15, 0}

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

    r2.initialize("package://r2_description/urdf/r2c6.urdf",  // urdf
                   "package://r2_moveit_config/config/r2.srdf",     // srdf
                   "package://r2_moveit_config/config/joint_limits.yaml",      // joint limits
                   "package://r2_moveit_config/config/kinematics.yaml"         // kinematics
    );

    Scene scene(r2);

    //add the handrails
    // scene.updateCollisionObject("ISS Handrail 0", ,);


    OMPL::OMPLPipelinePlanner planner(r2);
    planner.initialize("package://r2_moveit_config/config/ompl_planning.yaml",  // planner config
                       OMPL::Settings(), // settings
                       "ompl_interface/OMPLPlanningContextManager" // plugin
    );

    MotionRequestBuilder request(planner, "legsandtorso");
    //Lock the right foot in place:
    request.addPathPositionConstraint("r2/right_leg/gripper/tip", "/world", Eigen::Affine3d(Eigen::Translation3d(1.983001, 0.321150, -1.361)), Geometry(Geometry::ShapeType::Type::CYLINDER, Eigen::Vector3d(0.01,0.01,0.01), "right_foot_base_position"));

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

        // rviz.update(res);

        time_spent += (ros::Time::now().nsec-begin);
        ros::spinOnce();
        rate.sleep();
    }

    std::cout<<"Time spent: "<<time_spent<<std::endl;

    return 0;
}
