/* Author: Zachary Kingston */

#include <chrono>
#include <thread>

#include <robowflex_library/log.h>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/io.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using namespace robowflex;

int main(int /*argc*/, char ** /*argv*/)
{
    auto world = std::make_shared<darts::World>();

    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");
    auto fetch2 = fetch1->cloneRobot("fetch2");
    fetch2->setDof(4, 1);

    world->addRobot(fetch1);
    world->addRobot(fetch2);

    darts::Window window(world);

    darts::PlanBuilder builder(world);
    builder.addGroup("fetch1", "arm_with_torso");
    builder.addGroup("fetch2", "arm_with_torso");

    builder.setStartConfiguration({
        0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
        0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
    });

    builder.initialize();

    darts::TSR::Specification goal1_spec;
    goal1_spec.setFrame("fetch1", "wrist_roll_link", "base_link");
    goal1_spec.setPose(0.4, 0.3, 0.92,  //
                                        // 0.5, -0.5, 0.5, 0.5);
                       0.707, 0, 0, 0.707);

    darts::TSR::Specification goal2_spec;
    goal2_spec.setFrame("fetch2", "wrist_roll_link", "base_link");
    goal2_spec.setPose(0.4, -0.3, 0.92,  //
                       0.707, 0, 0, -0.707);

    auto goal1_tsr = std::make_shared<darts::TSR>(world, goal1_spec);
    auto goal2_tsr = std::make_shared<darts::TSR>(world, goal2_spec);

    auto goal = builder.getGoalTSR({goal1_tsr, goal2_tsr});
    builder.setGoal(goal);

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    rrt->setRange(1.);
    builder.ss->setPlanner(rrt);

    builder.setup();
    builder.ss->print();

    window.run([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        while (true)
        {
            goal->startSampling();
            ompl::base::PlannerStatus solved = builder.ss->solve(30.0);
            goal->stopSampling();

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (solved)
            {
                RBX_INFO("Found solution!");
                window.animatePath(builder, builder.getSolutionPath());
            }
            else
                RBX_WARN("No solution found");

            builder.ss->clear();
        }
    });

    return 0;
}
