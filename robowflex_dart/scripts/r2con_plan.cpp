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
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

using namespace robowflex;

static const std::string GROUP = "legsandtorso";
// static const std::string GROUP = "legs_no_world";

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

    const auto &mic = message.workspace_parameters.min_corner;
    const auto &mac = message.workspace_parameters.max_corner;
    world->getWorkspaceLow()[0] = mic.x;
    world->getWorkspaceLow()[1] = mic.y;
    world->getWorkspaceLow()[2] = mic.z;
    world->getWorkspaceHigh()[0] = mac.x;
    world->getWorkspaceHigh()[1] = mac.y;
    world->getWorkspaceHigh()[2] = mac.z;

    darts::PlanBuilder builder(world);
    builder.addGroup("r2", GROUP);

    for (std::size_t i = 0; i < message.start_state.joint_state.name.size(); ++i)
    {
        std::string name = message.start_state.joint_state.name[i];
        double value = message.start_state.joint_state.position[i];

        r2->setJoint(name, value);
    }

    builder.setStartConfigurationFromWorld();

    double pi = dart::math::constants<double>::pi();
    double inf = std::numeric_limits<double>::infinity();

    //
    // Waist TSR
    //
    darts::TSR::Specification waist_spec;
    waist_spec.setFrame("r2", "r2/waist_center", "r2/left_leg/gripper/tip");
    waist_spec.setRotation(0.999999999989, -2.52319271143e-06, 3.8366002265e-06, -6.53604813238e-07);
    waist_spec.setNoPosTolerance();

    auto waist_tsr = std::make_shared<darts::TSR>(world, waist_spec);
    waist_tsr->useGroup(GROUP);

    //
    // Left Leg TSR
    //
    darts::TSR::Specification lleg_spec;
    lleg_spec.setTarget("r2", "r2/left_leg/gripper/tip");
    lleg_spec.setPose(0.451508662827, 0.246606909363, -1.10409735323,  //
                      2.523198695e-06, -0.999999999982, 1.98040305765e-06, 5.16339177538e-06);
    auto lleg_tsr = std::make_shared<darts::TSR>(world, lleg_spec);
    lleg_tsr->useGroup(GROUP);

    //
    // Starting Right Leg TSR
    //
    darts::TSR::Specification start_spec;
    start_spec.setTarget("r2", "r2/right_leg/gripper/tip");
    start_spec.setPose(1.2, -0.248108850885, -1.10411526908,  //
                       4.90351543079e-06, -0.999999999961, 1.82668011027e-06, 7.14501707513e-06);
    auto start_tsr = std::make_shared<darts::TSR>(world, start_spec);
    start_tsr->useGroup(GROUP);

    //
    // Randomly sample goal
    //

    builder.addConstraint(lleg_tsr);
    builder.addConstraint(waist_tsr);

    builder.initialize();

    auto goal = builder.setGoalTSR(start_tsr);

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    builder.ss->setPlanner(rrt);

    builder.setup();

    std::thread t([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(30.0);
        goal->stopSampling();

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
