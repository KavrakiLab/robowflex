/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/gui.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    darts::IO::addPackage("r2_description",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_description/");
    darts::IO::addPackage("r2_moveit_config",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_moveit_config/");

    auto world = std::make_shared<darts::World>();

    auto r21 = darts::loadMoveItRobot("r2_1",                                     //
                                      "package://r2_description/urdf/r2c6.urdf",  //
                                      "package://r2_moveit_config/config/r2.srdf");

    world->addRobot(r21);

    int nr2 = 1;
    if (argc > 1)
        nr2 = atoi(argv[1]);

    if (nr2 > 1)
    {
        auto r22 = r21->cloneRobot("r2_2");
        world->addRobot(r22);
    }

    darts::PlanBuilder builder(world);
    builder.addGroup("r2_1", "body");
    if (nr2 > 1)
        builder.addGroup("r2_2", "body");

    builder.initialize();

    builder.sampleStartConfiguration();
    builder.sampleGoalConfiguration();

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info);
    rrt->setRange(2.);
    builder.ss->setPlanner(rrt);

    builder.setup();
    builder.ss->print();

    darts::Window window(world);
    window.run([&]() {
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        if (solved)
        {
            std::cout << "Found solution!" << std::endl;
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            std::cout << "No solution found" << std::endl;
    });

    return 0;
}
