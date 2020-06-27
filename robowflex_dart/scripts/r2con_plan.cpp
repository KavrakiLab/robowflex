/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <robowflex_library/tf.h>
#include <robowflex_library/io/yaml.h>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/joints.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/planning.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/tools/multiplan/ParallelPlan.h>

using namespace robowflex;

static const std::string GROUP = "legsandtorso";

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

    world->addRobot(r2);

    moveit_msgs::MotionPlanRequest message;
    IO::fromYAMLFile(message, "package://robowflex_library/yaml/r2_plan_waist.yml");

    darts::PlanBuilder builder(world);
    builder.addGroup("r2", GROUP, 2);

    builder.getWorkspaceBoundsFromMessage(message);
    builder.getStartFromMessage("r2", message);

    //
    // Waist TSR
    //
    darts::TSR::Specification waist_spec;
    waist_spec.setFrame("r2", "r2/waist_center", "r2/left_leg/gripper/tip");
    waist_spec.setRotation(0.999999999989, -2.52319271143e-06, 3.8366002265e-06, -6.53604813238e-07);
    waist_spec.setNoPosTolerance();
    // waist_spec.setNoZRotTolerance();

    auto waist_tsr = std::make_shared<darts::TSR>(world, waist_spec);
    builder.addConstraint(waist_tsr);

    //
    // Left Leg TSR
    //
    darts::TSR::Specification lleg_spec;
    lleg_spec.setTarget("r2", "r2/left_leg/gripper/tip");
    lleg_spec.setPose(0.451508662827, 0.246606909363, -1.10409735323,  //
                      2.523198695e-06, -0.999999999982, 1.98040305765e-06, 5.16339177538e-06);
    auto lleg_tsr = std::make_shared<darts::TSR>(world, lleg_spec);
    builder.addConstraint(lleg_tsr);

    //
    // Starting Right Leg TSR
    //
    darts::TSR::Specification start_spec;
    start_spec.setTarget("r2", "r2/right_leg/gripper/tip");
    start_spec.setPose(1.5, -0.248108850885, -1.10411526908,  //
                       4.90351543079e-06, -0.999999999961, 1.82668011027e-06, 7.14501707513e-06);
    auto start_tsr = std::make_shared<darts::TSR>(world, start_spec);
    builder.setGoalTSR(start_tsr);

    //
    // Initialize
    //
    builder.initialize();

    //
    // Setup planner
    //

    // ompl::tools::ParallelPlan pp(builder.ss->getProblemDefinition());
    // auto rrt1 = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    // rrt1->setRange(100);
    // auto rrt2 = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    // rrt2->setRange(100);
    // auto rrt3 = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    // rrt3->setRange(100);
    // auto rrt4 = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    // rrt4->setRange(100);

    // pp.addPlanner(rrt1);
    // pp.addPlanner(rrt2);
    // pp.addPlanner(rrt3);
    // pp.addPlanner(rrt4);

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    rrt->setRange(100);
    builder.ss->setPlanner(rrt);

    builder.setup();

    std::thread t([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        while (true)
        {
            builder.goal_tsr->startSampling();
            ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
            // ompl::base::PlannerStatus solved = pp.solve(60.0, 1, 1, true);
            builder.goal_tsr->stopSampling();

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
