#include <thread>
#include <chrono>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <robowflex_library/util.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/gui.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    //
    // Standard Robowflex setup
    // Create the default Fetch robot and scene.
    //
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch.yml");

    //
    // Convert to Dart
    //
    auto fetch_dart = std::make_shared<darts::Robot>(fetch);
    auto fetch_name = fetch_dart->getName();
    auto scene_dart = std::make_shared<darts::Structure>("scene", scene);
    auto ground = std::make_shared<darts::Structure>("ground");
    ground->addGround(-0.2);

    // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addStructure(scene_dart);
    world->addStructure(ground);

    darts::Window window(world);

    const auto &planToPick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(-0.2, 0.6, 0.92,  //
                           0.5, -0.5, 0.5, 0.5);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();
        builder.setStartConfigurationFromWorld();

        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.4, 0.6, 0.96,  //
                          0.5, -0.5, 0.5, 0.5);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            std::cout << "Found solution!" << std::endl;
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            std::cout << "No solution found" << std::endl;
    };

    const auto &planToPlace = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        builder.setStartConfigurationFromWorld();

        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(-0.2, 0.6, 0.92,  //
                          0.5, -0.5, 0.5, 0.5);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            std::cout << "Found solution!" << std::endl;
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            std::cout << "No solution found" << std::endl;
    };

    window.run([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        planToPick();

        auto cube = scene_dart->getFrame("Cube3");
        fetch_dart->reparentFreeFrame(cube, "wrist_roll_link");

        planToPlace();
    });

    return 0;
}
