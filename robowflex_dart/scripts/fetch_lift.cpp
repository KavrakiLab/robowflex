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

using namespace robowflex;

int main(int argc, char **argv)
{
    //
    // Load and setup the fetch
    //

    double radius = 0.8;
    auto world = std::make_shared<darts::World>();
    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");
    auto fetch2 = fetch1->cloneRobot("fetch2");
    fetch2->setDof(2, -1.57);
    fetch2->setDof(3, radius);
    fetch2->setDof(4, radius);

    auto fetch3 = fetch1->cloneRobot("fetch3");
    fetch3->setDof(2, 1.57);
    fetch3->setDof(3, radius);
    fetch3->setDof(4, -radius);

    auto fetch4 = fetch1->cloneRobot("fetch4");
    fetch4->setDof(2, 3.14);
    fetch4->setDof(3, radius * 2);

    // make box

    auto scene = std::make_shared<darts::Structure>("object");

    dart::dynamics::FreeJoint::Properties joint;
    joint.mName = "box";
    joint.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(radius, 0, 1.8);

    auto pair = scene->addFreeFrame(joint, darts::makeBox(0.50, 0.50, 0.1));

    world->addRobot(fetch1);
    world->addRobot(fetch2);
    world->addRobot(fetch3);
    world->addRobot(fetch4);
    world->addStructure(scene);

    //
    // Initial goal - touch fingertips
    //
    const auto &touch = [&] {
        darts::PlanBuilder builder(world);
        builder.addGroup("fetch1", "arm_with_torso");
        builder.addGroup("fetch2", "arm_with_torso");
        builder.addGroup("fetch3", "arm_with_torso");
        builder.addGroup("fetch4", "arm_with_torso");

        builder.setStartConfiguration({
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
            0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
        });

        builder.initialize();
        darts::TSR::Specification goal1_spec;
        goal1_spec.setFrame("fetch1", "wrist_roll_link", "base_link");
        goal1_spec.setPose(0.5, 0, 1.7,  //
                           0.707, 0, -0.707, 0);

        darts::TSR::Specification goal2_spec;
        goal2_spec.setFrame("fetch2", "wrist_roll_link", "base_link");
        goal2_spec.setPose(0.5, 0, 1.7,  //
                           0.707, 0, -0.707, 0);

        darts::TSR::Specification goal3_spec;
        goal3_spec.setFrame("fetch3", "wrist_roll_link", "base_link");
        goal3_spec.setPose(0.5, 0, 1.7,  //
                           0.707, 0, -0.707, 0);

        darts::TSR::Specification goal4_spec;
        goal4_spec.setFrame("fetch4", "wrist_roll_link", "base_link");
        goal4_spec.setPose(0.5, 0, 1.7,  //
                           0.707, 0, -0.707, 0);

        auto goal1_tsr = std::make_shared<darts::TSR>(world, goal1_spec);
        auto goal2_tsr = std::make_shared<darts::TSR>(world, goal2_spec);
        auto goal3_tsr = std::make_shared<darts::TSR>(world, goal3_spec);
        auto goal4_tsr = std::make_shared<darts::TSR>(world, goal4_spec);

        auto goal = builder.getGoalTSR({goal1_tsr, goal2_tsr, goal3_tsr, goal4_tsr});
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        // auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info);
        // auto rrt = std::make_shared<ompl::geometric::RRT>(builder.info);
        rrt->setRange(100.);
        builder.ss->setPlanner(rrt);
        // auto kpiece = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        // builder.ss->setPlanner(kpiece);
        // auto prm = std::make_shared<ompl::geometric::PRM>(builder.info);
        // builder.ss->setPlanner(prm);

        builder.setup();

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
        builder.addGroup("fetch3", "arm_with_torso");
        builder.addGroup("fetch4", "arm_with_torso");

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification con1_spec;
        con1_spec.setTarget("fetch2", "wrist_roll_link");
        con1_spec.setBase("fetch1", "wrist_roll_link");
        con1_spec.setPoseFromWorld(world);
        auto con1_tsr = std::make_shared<darts::TSR>(world, con1_spec);
        builder.addConstraint(con1_tsr);

        darts::TSR::Specification con2_spec;
        con2_spec.setTarget("fetch3", "wrist_roll_link");
        con2_spec.setBase("fetch1", "wrist_roll_link");
        con2_spec.setPoseFromWorld(world);
        auto con2_tsr = std::make_shared<darts::TSR>(world, con2_spec);
        builder.addConstraint(con2_tsr);

        darts::TSR::Specification con3_spec;
        con3_spec.setTarget("fetch4", "wrist_roll_link");
        con3_spec.setBase("fetch1", "wrist_roll_link");
        con3_spec.setPoseFromWorld(world);
        auto con3_tsr = std::make_shared<darts::TSR>(world, con3_spec);
        builder.addConstraint(con3_tsr);

        builder.options.constraints.delta = 0.4;
        builder.options.constraints.lambda = 2;
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame("fetch1", "wrist_roll_link", "base_link");
        goal_spec.setPose(0.4, 0, 0.7,  //
                          0.707, 0, -0.707, 0);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(1.);
        builder.ss->setPlanner(rrt);

        // auto kpiece = std::make_shared<ompl::geometric::BKPIECE1>(builder.info);
        // kpiece->setRange(2);
        // builder.ss->setPlanner(kpiece);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(120.0);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        touch();
        auto cube = scene->getFrame("box");
        fetch1->reparentFreeFrame(cube, "wrist_roll_link");
        dance();
    });

    world->openOSGViewer();
    return 0;
}
