/* Author: Zachary Kingston */

#include <chrono>
#include <thread>

#include <robowflex_library/log.h>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/io.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/world.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    auto world = std::make_shared<darts::World>();

    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");

    world->addRobot(fetch1);

    int nfetch = 1;
    if (argc > 1)
        nfetch = atoi(argv[1]);

    if (nfetch > 1)
    {
        auto fetch2 = fetch1->cloneRobot("fetch2");
        fetch2->setDof(2, -1.57);
        fetch2->setDof(3, 0.525);
        fetch2->setDof(4, 0.525);
        world->addRobot(fetch2);
    }

    if (nfetch > 2)
    {
        auto fetch3 = fetch1->cloneRobot("fetch3");
        fetch3->setDof(2, 1.57);
        fetch3->setDof(3, 0.525);
        fetch3->setDof(4, -0.525);
        world->addRobot(fetch3);
    }

    if (nfetch > 3)
    {
        auto fetch4 = fetch1->cloneRobot("fetch4");
        fetch4->setDof(2, 3.14);
        fetch4->setDof(3, 1.05);
        world->addRobot(fetch4);
    }

    darts::Window window(world);
    window.run([&] {
        darts::PlanBuilder builder(world);
        builder.addGroup("fetch1", "arm_with_torso");
        if (nfetch > 1)
            builder.addGroup("fetch2", "arm_with_torso");
        if (nfetch > 2)
            builder.addGroup("fetch3", "arm_with_torso");
        if (nfetch > 3)
            builder.addGroup("fetch4", "arm_with_torso");

        builder.setStartConfiguration({
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,
        });

        builder.initialize();

        auto goal = builder.getGoalConfiguration({
            0.0,  0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,
            0.13, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
            0.38, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
            0.26, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
        });
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(1.);
        builder.ss->setPlanner(rrt);

        builder.setup();
        builder.ss->print();

        ompl::base::PlannerStatus solved = builder.ss->solve(30.0);

        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution found");
    });

    return 0;
}
