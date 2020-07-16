#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/detail/r2.h>

using namespace robowflex;

int dumpTransform()
{
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({});
    r2->setStateFromYAMLFile("package://robowflex_library/yaml/r2_state.yml");
    r2->dumpGeometry("r2.yml");
    r2->dumpTransforms("r2_state.yml");

    return 0;
}

int planFromFile()
{
    // Create an R2 robot, initialize the `legsandtorso` kinematics solver.
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({"legsandtorso"});

    // Create an RViz visualizer
    IO::RVIZHelper rviz(r2);

    // Dump the geometry information for blender visualization.
    r2->dumpGeometry("r2.yml");

    // Load the ISS from a world file.
    auto iss_scene = std::make_shared<Scene>(r2);
    iss_scene->fromYAMLFile("package://robowflex_library/yaml/r2_world.yml");

    // Display the scene geometry in RViz.
    rviz.updateScene(iss_scene);

    // Create the default motion planner for R2.
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    planner->initialize();

    // Load a motion planning request (a step with a torso constraint).
    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan_waist.yml");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(iss_scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Display the planned trajectory in RViz.
    rviz.updateTrajectory(res);

    // Spin once to let messages escape.
    ros::spinOnce();

    // Dump path transforms for visualization in blender.
    r2->dumpPathTransforms(*res.trajectory_, "r2_path.yml", 30, 0.5);

    return 0;
}

int planAndBuild()
{
    // Create an R2 robot, initialize the `legsandtorso` kinematics solver.
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({"legsandtorso"});

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(r2);

    // Create the default motion planner for R2.
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    planner->initialize();

    // Load a motion planning request (a step with a torso constraint, we only want the start state).
    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan_waist.yml");

    // Clear path constraints so we can rebuild them.
    request.getPathConstraints().position_constraints.clear();
    request.getPathConstraints().orientation_constraints.clear();

    // Set the scratch state of the robot.
    r2->setState(request.getRequest().start_state);

    const std::string world = "world";
    const std::string waist = "r2/waist_center";
    const std::string left_foot = "r2/left_leg/gripper/tip";
    const std::string right_foot = "r2/right_leg/gripper/tip";

    Eigen::Vector3d tolerance_feet(0.01, 0.01, 0.01);
    Eigen::Vector3d tolerance_waist(0.005, 0.005, 0.005);
    // Set a goal region to plan to.
    request.setGoalRegion(                                                                 //
        right_foot, world,                                                                 //
        RobotPose(Eigen::Translation3d(1.26, -0.248, -1.104)), Geometry::makeSphere(0.1),  //
        Eigen::Quaterniond(0, 0, 1, 0), Eigen::Vector3d{0.01, 0.01, 0.01});

    // Set a pose constraint on the left foot (keep fixed throughout the path).
    auto foot_tf = r2->getLinkTF(left_foot);
    request.addPathPoseConstraint(            //
        left_foot, world,                     //
        foot_tf, Geometry::makeSphere(0.01),  //
        Eigen::Quaterniond(foot_tf.rotation()), tolerance_feet);

    // Set a orientation constraint on the waist (to keep it up throughout the path)
    auto waist_tf = r2->getRelativeLinkTF(left_foot, waist);
    request.addPathOrientationConstraint(  //
        waist, left_foot,                  //
        Eigen::Quaterniond(waist_tf.rotation()), tolerance_waist);

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    return 0;
}

int main(int argc, char **argv)
{
    // Startup ROS.
    ROS ros(argc, argv);

    // Dump a state for animation.
    dumpTransform();

    // Plan using configuration from files.
    planFromFile();

    // Plan by building a motion request.
    // planAndBuild();
}
