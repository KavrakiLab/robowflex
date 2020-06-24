/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/planning.h>

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

    auto r2 = darts::loadMoveItRobot("r2",                                       //
                                     "package://r2_description/urdf/r2c6.urdf",  //
                                     "package://r2_moveit_config/config/r2.srdf");

    r2->setDof(4, -0.3);
    r2->setDof(5, 1.5);

    world->addRobot(r2);

    darts::PlanBuilder builder(world);
    builder.addGroup("r2", "left_arm_waist");
    builder.addGroup("r2", "right_arm");

    builder.initialize();

    builder.sampleStartConfiguration();
    auto goal = builder.sampleGoalConfiguration();
    builder.setGoal(goal);

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    rrt->setRange(1.);
    builder.ss->setPlanner(rrt);

    builder.setup();
    builder.ss->print();

    ompl::base::PlannerStatus solved = builder.ss->solve(10.0);

    std::thread t([&]() {
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            builder.animateSolutionInWorld();
        }
        else
            std::cout << "No solution found" << std::endl;
    });

    world->openOSGViewer();
    return 0;
}
