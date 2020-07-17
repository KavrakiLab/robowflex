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
#include <robowflex_dart/gui.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

using namespace robowflex;

// static const std::string GROUP = "body";
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

    darts::PlanBuilder builder(world);
    auto goal = builder.fromMessage("r2", message);
    auto tsrgoal = std::dynamic_pointer_cast<darts::TSRGoal>(goal);

    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
    rrt->setRange(100);
    builder.ss->setPlanner(rrt);

    builder.setup();

    darts::Window window(world);
    window.run([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        while (true)
        {
            tsrgoal->startSampling();
            ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
            tsrgoal->stopSampling();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (solved)
            {
                std::cout << "Found solution!" << std::endl;
                window.animatePath(builder, builder.getSolutionPath());
            }
            else
                std::cout << "No solution found" << std::endl;

            builder.ss->clear();
        }
    });

    return 0;
}
