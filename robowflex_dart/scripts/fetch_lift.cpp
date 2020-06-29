/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/planning.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    //
    // Load and setup the fetch
    //

    double height = 0.6;
    double raise = 1.2;
    double radius = 0.8;
    auto world = std::make_shared<darts::World>();
    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");
    auto fetch2 = fetch1->cloneRobot("fetch2");
    fetch2->setDof(2, -1.57);
    fetch2->setDof(3, radius);
    fetch2->setDof(4, radius);

    // make box
    auto scene = std::make_shared<darts::Structure>("object");

    dart::dynamics::FreeJoint::Properties joint;
    joint.mName = "box";
    joint.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(radius, 0, height);

    scene->addFreeFrame(joint, darts::makeBox(0.50, 0.50, 0.1));

    world->addRobot(fetch1);
    world->addRobot(fetch2);
    world->addStructure(scene);

    const auto &touch = [&] {
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
        goal1_spec.setPose(0.5, 0, height - 0.1,  //
                           0.707, 0, -0.707, 0);

        darts::TSR::Specification goal2_spec;
        goal2_spec.setFrame("fetch2", "wrist_roll_link", "base_link");
        goal2_spec.setPose(0.5, 0, height - 0.1,  //
                           0.707, 0, -0.707, 0);

        auto goal1_tsr = std::make_shared<darts::TSR>(world, goal1_spec);
        auto goal2_tsr = std::make_shared<darts::TSR>(world, goal2_spec);
        auto goal = builder.getGoalTSR({goal1_tsr, goal2_tsr});
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(1.);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->options.max_samples = 1;
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            builder.animateSolutionInWorld(1);
        }
        else
            std::cout << "No solution found" << std::endl;

        return solved;
    };

    const auto &dance = [&] {
        darts::PlanBuilder builder(world);
        builder.addGroup("fetch1", "arm_with_torso");
        builder.addGroup("fetch2", "arm_with_torso");

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification con1_spec;
        con1_spec.setTarget("fetch2", "wrist_roll_link");
        con1_spec.setBase("fetch1", "wrist_roll_link");
        con1_spec.setPoseFromWorld(world);
        auto con1_tsr = std::make_shared<darts::TSR>(world, con1_spec);
        builder.addConstraint(con1_tsr);

        darts::TSR::Specification con2_spec;
        con2_spec.setFrame("fetch1", "wrist_roll_link", "base_link");
        con2_spec.setPoseFromWorld(world);
        con2_spec.setNoPosTolerance();
        con2_spec.setXRotTolerance(0.05);
        con2_spec.setYRotTolerance(0.05);
        con2_spec.setNoZRotTolerance();

        auto con2_tsr = std::make_shared<darts::TSR>(world, con2_spec);
        builder.addConstraint(con2_tsr);

        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame("fetch1", "wrist_roll_link", "base_link");
        goal_spec.setPose(0.4, 0, height + raise,  //
                          0.707, 0, -0.707, 0);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(100.);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->options.max_samples = 100;
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(240.0);
        goal->stopSampling();

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            builder.animateSolutionInWorld(0);
        }
        else
            std::cout << "No solution found" << std::endl;

        return solved;
    };

    std::thread t([&]() {
        std::cout << "Press enter" << std::endl;
        std::cin.ignore();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        touch();
        auto cube = scene->getFrame("box");
        fetch1->reparentFreeFrame(cube, "wrist_roll_link");
        dance();
    });

    world->openOSGViewer();
    return 0;
}
