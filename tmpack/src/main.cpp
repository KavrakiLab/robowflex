#include "footstep_planner/my_walker.cpp"
#include <include/robowflex.h>
#include <ros/ros.h>
#include <signal.h>

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
    planner.initialize("package://r2_moveit_config/config/ompl_planning.yaml"  // planner config
    );

    MotionRequestBuilder request(planner, "legsandtorso");
    request.setStartConfiguration({0,0,0,0,0,0,1,0,0,8.88178e-16,1.5708,8.88178e-16,1.5708,1.5708,0,0,0,0,8.88178e-16,0,1.5708,0,1.5708,1.5708,0,0,0,0,0.872665,-1.39626,-1.8326,-2.44346,1.39626,0,0,8.88178e-16,0,8.88178e-16,8.88178e-16,0,8.88178e-16,0,0,8.88178e-16,8.88178e-16,-8.88178e-16,8.88178e-16,8.88178e-16,0,1.77636e-15,1.77636e-15,-1.77636e-15,0,-0.872665,-1.39626,1.8326,-2.44346,-1.39626,0,8.88178e-16,0,0,0,-8.88178e-16,0,8.88178e-16,0,0,-8.88178e-16,0,0,8.88178e-16,0,8.88178e-16,1.77636e-15,0,-8.88178e-16,0,-0.0872665,8.88178e-16,8.88178e-16});
    request.setGoalConfiguration({0,0,0,0,0,0,1,0,0,8.88178e-16,1.5708,8.88178e-16,1.5708,1.5708,0,0,0,0,8.88178e-16,0,1.5708,0,1.5708,1.5708,0,0,0,0,0.872665,-1.39626,-1.8326,-2.44346,1.39626,0,0,8.88178e-16,0,8.88178e-16,8.88178e-16,0,8.88178e-16,0,0,8.88178e-16,8.88178e-16,-8.88178e-16,8.88178e-16,8.88178e-16,0,1.77636e-15,1.77636e-15,-1.77636e-15,0,-0.872665,-1.39626,1.8326,-2.44346,-1.39626,0,8.88178e-16,0,0,0,-8.88178e-16,0,8.88178e-16,0,0,-8.88178e-16,0,0,8.88178e-16,0,8.88178e-16,1.77636e-15,0,-8.88178e-16,0,-0.0772665,8.88178e-16,8.88178e-16});

    std::vector<double> start = {0,0,0,0,0,0,1,0,0,8.88178e-16,1.5708,8.88178e-16,1.5708,1.5708,0,0,0,0,8.88178e-16,0,1.5708,0,1.5708,1.5708,0,0,0,0,0.872665,-1.39626,-1.8326,-2.44346,1.39626,0,0,8.88178e-16,0,8.88178e-16,8.88178e-16,0,8.88178e-16,0,0,8.88178e-16,8.88178e-16,-8.88178e-16,8.88178e-16,8.88178e-16,0,1.77636e-15,1.77636e-15,-1.77636e-15,0,-0.872665,-1.39626,1.8326,-2.44346,-1.39626,0,8.88178e-16,0,0,0,-8.88178e-16,0,8.88178e-16,0,0,-8.88178e-16,0,0,8.88178e-16,0,8.88178e-16,1.77636e-15,0,-8.88178e-16,0,-0.0872665,8.88178e-16,8.88178e-16};

    MyWalker walker(r2, "legsandtorso", planner, scene, request, start);

    ros::Rate rate(0.5);
    while (ros::ok())
    {

        auto res = walker.plan();
        // auto res = planner.plan(scene, request.getRequest());
        // if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        //     break;

        // rviz.update(res);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
