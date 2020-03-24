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
    se3ez::io::addPackage("fetch_description",  //
                          "/home/zak/ros/melodic/system/src/fetch_description/");
    se3ez::io::addPackage("fetch_moveit_config",  //
                          "/home/zak/ros/melodic/system/src/fetch_moveit_config/");

    auto world = std::make_shared<se3ez::World>();

    auto fetch1 = se3ez::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");
    world->addRobot(fetch1);

    int nfetch = 1;
    if (argc > 1)
        nfetch = atoi(argv[1]);

    if (nfetch > 1)
    {
        auto fetch2 = fetch1->clone("fetch2");
        fetch2->setDof(2, -1.57);
        fetch2->setDof(3, 0.525);
        fetch2->setDof(4, 0.525);
        world->addRobot(fetch2);
    }

    if (nfetch > 2)
    {
        auto fetch3 = fetch1->clone("fetch3");
        fetch3->setDof(2, 1.57);
        fetch3->setDof(3, 0.525);
        fetch3->setDof(4, -0.525);
        world->addRobot(fetch3);
    }

    if (nfetch > 3)
    {
        auto fetch4 = fetch1->clone("fetch4");
        fetch4->setDof(2, 3.14);
        fetch4->setDof(3, 1.05);
        world->addRobot(fetch4);
    }

    auto space = std::make_shared<se3ez::StateSpace>(world);
    space->addGroup("fetch1", "arm_with_torso");
    if (nfetch > 1)
        space->addGroup("fetch2", "arm_with_torso");
    if (nfetch > 2)
        space->addGroup("fetch3", "arm_with_torso");
    if (nfetch > 3)
        space->addGroup("fetch4", "arm_with_torso");

    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker([&](const ompl::base::State *state) {
        space->setWorldState(state);
        return not world->inCollision();
    });

    ompl::base::ScopedState<> start(space);
    // start = {
    //     0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
    // };

    start = {
        0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
        0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
        0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,  //
        0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0,
    };
    std::cout << ss.getSpaceInformation()->isValid(start.get()) << std::endl;

    // create a random goal state
    ompl::base::ScopedState<> goal(space);
    // goal = {
    //     0.300, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
    // };
    goal = {
        0.0,  0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,
        0.13, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
        0.38, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
        0.26, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007,  //
    };

    ss.setStartAndGoalStates(start, goal);
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation(), true);
    rrt->setRange(1.);
    ss.setPlanner(rrt);
    // auto prm = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    // ss.setPlanner(prm);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    ompl::base::PlannerStatus solved = ss.solve(10.0);

    std::thread t([&]() {
        // attempt to solve the problem within one second of planning time
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
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
