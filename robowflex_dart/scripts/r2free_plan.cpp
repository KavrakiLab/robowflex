/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <se3ez/io.h>
#include <se3ez/robot.h>
#include <se3ez/space.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

int main(int argc, char **argv)
{
    se3ez::io::addPackage("r2_description",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_description/");
    se3ez::io::addPackage("r2_moveit_config",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_moveit_config/");

    auto world = std::make_shared<se3ez::World>();

    auto r21 = se3ez::loadMoveItRobot("r2_1",                                     //
                                      "package://r2_description/urdf/r2c6.urdf",  //
                                      "package://r2_moveit_config/config/r2.srdf");

    world->addRobot(r21);
    auto space = std::make_shared<se3ez::StateSpace>(world);
    space->addGroup("r2_1", "body");

    // auto r22 = r21->clone("r2_2");
    // world->addRobot(r22);
    // space->addGroup("r2_2", "body");

    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker([&](const ompl::base::State *state) {
        space->setWorldState(state);
        return not world->inCollision();
    });

    auto sampler = space->allocStateSampler();

    ompl::base::ScopedState<> start(space);
    do
    {
        sampler->sampleUniform(start.get());
        space->enforceBounds(start.get());
        space->printState(start.get(), std::cout);
    } while (not ss.getSpaceInformation()->isValid(start.get()));
    // // start = {};

    ompl::base::ScopedState<> goal(space);
    do
    {
        sampler->sampleUniform(goal.get());
    } while (not ss.getSpaceInformation()->isValid(goal.get()));
    // // goal = {};

    ss.setStartAndGoalStates(start, goal);
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation(), true);
    rrt->setRange(4.);
    ss.setPlanner(rrt);

    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = ss.solve(60.0);

    std::thread t([&]() {
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            for (unsigned int i = 0; i < 10; ++i)
                ss.simplifySolution();

            auto path = ss.getSolutionPath();
            path.interpolate();
            path.print(std::cout);

            while (true)
            {
                space->setWorldState(path.getStates()[0]);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                for (const auto &state : path.getStates())
                {
                    space->setWorldState(state);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        else
            std::cout << "No solution found" << std::endl;
    });

    world->openOSGViewer();
    return 0;
}
