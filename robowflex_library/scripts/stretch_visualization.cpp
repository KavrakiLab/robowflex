/* Author: Carlos Quintero-Peña */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/stretch.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file stretch_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * Stretch robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, start/goal states, and motion plan
 * displayed in RViz.
 */

static const std::string GROUP = "mobile_base_manipulator";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Stretch robot.
    auto stretch = std::make_shared<StretchRobot>();
    stretch->initialize(true);

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(stretch);
    IO::RobotBroadcaster bc(stretch);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(stretch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_stretch.yml");

    // Create the default planner for the Stretch.
    auto planner = std::make_shared<OMPL::StretchOMPLPipelinePlanner>(stretch, "default");
    planner->initialize();
    
//     stretch->setBasePose(1.0, 0.0, 0.0);
    
    // Visualize the scene (start state) in RViz.
    scene->getCurrentState() = *stretch->getScratchState();
    rviz.updateScene(scene);
    ROS_INFO("Visualizing start state");
    std::cin.get();
    
    // Create a motion planning request.
    MotionRequestBuilder request(planner, GROUP);
    stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0});  // home
    
    std::cout << "Number of dofs of this group " << stretch->getScratchState()->getJointModelGroup(GROUP)->getVariableCount() << std::endl;
    for (const auto &name : stretch->getScratchState()->getJointModelGroup(GROUP)->getActiveJointModelNames())
        std::cout << name << std::endl;
    
    const auto &solver = stretch->getScratchState()->getJointModelGroup(GROUP)->getSolverInstance();
    if (solver)
        std::cout << "Pointer is not null!" << std::endl;
    
    auto frames = solver->getTipFrame();
    std::cout << frames << std::endl;
    
    std::cin.get();
    
    // Set start configuration.
    request.setStartConfiguration(stretch->getScratchState());

    // Create IK query.
    auto query =
        Robot::IKQuery(GROUP, "link_wrist_yaw", *stretch->getScratchState(), Eigen::Vector3d{-0.2, 0.23, -0.71});
    query.scene = scene;
    auto tips(query.tips);
    std::cout << "Number of tips " << tips.size() << ", tip: " << tips[0] << std::endl;
//     std::cout << stretch->getSolverTipFrames(GROUP)[0] << std::endl;
    std::cout << stretch->getSolverTipFrames("stretch_arm")[0] << std::endl;
    
    std::cin.get();
    
    if (not stretch->setFromIK(query))
    {
        RBX_ERROR("IK solution not found");
        return 1;
    }
    
    // Set goal configuration.
    request.setGoalConfiguration(stretch->getScratchState());

    // Visualize the goal state in RViz.
    scene->getCurrentState() = *stretch->getScratchState();
    rviz.updateScene(scene);
    ROS_INFO("Visualizing goal state");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz.
    rviz.updateTrajectory(res);
    RBX_INFO("Visualizing trajectory.");
    std::cin.get();

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("stretch_pick.yml");

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
