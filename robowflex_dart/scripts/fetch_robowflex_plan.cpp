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
    auto scene_dart = std::make_shared<darts::Structure>("scene", scene);
    auto ground = std::make_shared<darts::Structure>("ground");
    ground->addGround(-0.2);

    // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addStructure(scene_dart);
    world->addStructure(ground);

    darts::PlanBuilder builder(world);
    builder.addGroup(fetch_dart->getName(), GROUP);

    // get start/ goal
    darts::TSR start_tsr(                            //
        fetch_dart, "wrist_roll_link", "base_link",  //
        Eigen::Vector3d{-0.2, 0.6, 0.92},            //
        Eigen::Quaterniond{0.5, -0.5, 0.5, 0.5});
    start_tsr.useGroup(GROUP);
    start_tsr.solve();
    builder.setStartConfigurationFromWorld();
    std::cout << start_tsr.distance() << std::endl;

    darts::TSR goal_tsr(                             //
        fetch_dart, "wrist_roll_link", "base_link",  //
        Eigen::Vector3d{0.5, 0.6, 0.92},             //
        Eigen::Quaterniond{0.5, -0.5, 0.5, 0.5});
    goal_tsr.useGroup(GROUP);
    goal_tsr.solve();
    builder.setGoalConfigurationFromWorld();
    std::cout << goal_tsr.distance() << std::endl;

    // Create constraint
    double table = 0.5;
    double pi = dart::math::constants<double>::pi();
    double alpha = 1e-3;                               // pi / 16.;
    double beta = 1e-3;                                // pi;
    // double beta = pi;                                // pi;
    auto tsr = std::make_shared<darts::TSR>(           //
        fetch_dart, "wrist_roll_link", "base_link",    //
        TF::createPoseQ(                               //
            Eigen::Vector3d{0.3, 0.8, 0.92},           //
            Eigen::Quaterniond{0.5, -0.5, 0.5, 0.5}),  //
        Eigen::Vector3d{table, table, 1e-3},           //
        Eigen::Vector3d{alpha, alpha, beta});
    builder.addConstraint(tsr);
    std::cout << tsr->distance() << std::endl;

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
