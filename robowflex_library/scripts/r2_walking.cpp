#include <boost/format.hpp>

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/path.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/detail/r2.h>

using namespace robowflex;

int plan()
{
    // Create an R2 robot, initialize the `legsandtorso` kinematics solver.
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({"legsandtorso"});
    r2->dumpGeometry("r2.yml");

    // Create an RViz visualizer
    IO::RVIZHelper rviz(r2);

    // Load the ISS from a world file.
    auto iss_scene = std::make_shared<Scene>(r2);
    iss_scene->fromYAMLFile("package://robowflex_library/yaml/bench/world.yml");

    // Display the scene geometry in RViz.
    rviz.removeScene();
    rviz.updateScene(iss_scene);

    // Create the default motion planner for R2.
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    planner->initialize();

    std::vector<planning_interface::MotionPlanResponse> responses;

    robot_state::RobotStatePtr first;

    for (unsigned int i = 1; i <= 6; ++i)
    {
        // Load a motion planning request (a step with a torso constraint).
        auto request = std::make_shared<MotionRequestBuilder>(planner, "legsandtorso");
        request->fromYAMLFile(
            boost::str(boost::format{"package://robowflex_library/yaml/bench/r2_%1%.yml"} % i));

        // Use previous state for start state
        if (i > 1)
            request->setStartConfiguration(r2->getScratchState());


        // Set some parameters
        // request->setConfig("RRTConnect_Constrained");
        request->setConfig("CBiRRT2");
        request->setAllowedPlanningTime(60);
        request->getRequest().num_planning_attempts = 1;

        // Update the displayed state in the scene
        r2->setState(request->getRequestConst().start_state);

        if (i == 1)
            first = std::make_shared<robot_state::RobotState>(*r2->getScratchState());

        iss_scene->getScene()->setCurrentState(*r2->getScratchState());
        rviz.updateScene(iss_scene);

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(iss_scene, request->getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;

        // Save end state to a file
        r2->setState(path::getFinalPositions(*res.trajectory_));
        r2->toYAMLFile(boost::str(boost::format{"r2_%1%_end.yml"} % i));

        // Display the planned trajectory in RViz.
        rviz.updateTrajectory(res);
        responses.push_back(res);

        // Dump the path to file
        r2->dumpPathTransforms(*res.trajectory_, boost::str(boost::format{"r2_path_%1%.yml"} % i), 60, 0.5);
    }

    // Update the displayed state in the scene
    iss_scene->getScene()->setCurrentState(*first);
    rviz.updateScene(iss_scene);

    rviz.updateTrajectories(responses);

    return 0;
}

int main(int argc, char **argv)
{
    // Startup ROS.
    ROS ros(argc, argv);
    plan();
}
