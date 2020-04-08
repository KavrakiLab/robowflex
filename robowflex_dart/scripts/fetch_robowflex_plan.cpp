#include <thread>
#include <chrono>

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

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

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Standard Robowflex setup
    // Create the default Fetch robot and scene.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch.yml");

    // Convert to Dart
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

    darts::PlanBuilder builder(world);
    builder.addGroup(fetch_name, GROUP);

    //
    // Sample Start from TSR
    //
    darts::TSR::Specification start_spec;
    start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
    start_spec.setPose(-0.2, 0.6, 0.92,  //
                       0.5, -0.5, 0.5, 0.5);

    darts::TSR start_tsr(world, start_spec);
    start_tsr.useGroup(GROUP);
    start_tsr.solveWorld();
    builder.setStartConfigurationFromWorld();

    //
    // Sample Goal from TSR
    //
    darts::TSR::Specification goal_spec;
    goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
    goal_spec.setPose(0.5, 0.6, 0.92,  //
                      0.5, -0.5, 0.5, 0.5);

    darts::TSR goal_tsr(world, goal_spec);
    goal_tsr.useGroup(GROUP);
    goal_tsr.solveWorld();
    builder.setGoalConfigurationFromWorld();

    //
    // Create constraint
    //
    darts::TSR::Specification constraint_spec;
    constraint_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
    constraint_spec.setPose(0.3, 0.8, 0.92,  //
                            0.5, -0.5, 0.5, 0.5);

    double table = 0.5;
    constraint_spec.setXPosTolerance(-table, table);
    constraint_spec.setYPosTolerance(-table, table);

    auto constraint_tsr = std::make_shared<darts::TSR>(world, constraint_spec);
    constraint_tsr->useGroup(GROUP);

    builder.addConstraint(constraint_tsr);

    // initialize and setup
    builder.initialize();

    // auto prm = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    // builder.ss->setPlanner(prm);

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    rrt->setRange(2);
    builder.ss->setPlanner(rrt);

    // auto rrt = std::make_shared<ompl::geometric::RRTstar>(builder.info);
    // rrt->setRange(2);
    // builder.ss->setPlanner(rrt);

    // auto bit = std::make_shared<ompl::geometric::BITstar>(builder.info);
    // builder.ss->setPlanner(bit);

    builder.setup();

    if (builder.ss->getOptimizationObjective())
        builder.ss->getOptimizationObjective()->setCostThreshold(
            ompl::base::Cost(std::numeric_limits<double>::infinity()));

    std::thread t([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        while (true)
        {
            ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (solved)
            {
                std::cout << "Found solution!" << std::endl;
                builder.animateSolutionInWorld(1);
            }
            else
                std::cout << "No solution found" << std::endl;

            builder.ss->clear();
        }
    });

    world->openOSGViewer();
    return 0;
}
