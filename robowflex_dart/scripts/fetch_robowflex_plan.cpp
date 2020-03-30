#include <thread>
#include <chrono>

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <robowflex_library/util.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

void getInitialConfiguration(darts::RobotPtr fetch_dart)
{
    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{-0.15, 0.6, 0.92});
    Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
    pose.rotate(orn);

    darts::TSR tsr(fetch_dart, "wrist_roll_link", "base_link");
    tsr.setPose(pose);
    tsr.useGroup(GROUP);

    tsr.solve();
}

void getFinalConfiguration(darts::RobotPtr fetch_dart)
{
    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{0.48, 0.6, 0.92});
    Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
    pose.rotate(orn);

    darts::TSR tsr(fetch_dart, "wrist_roll_link", "base_link");
    tsr.setPose(pose);
    tsr.useGroup(GROUP);

    tsr.solve();
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Standard Robowflex setup
    // Create the default Fetch robot and scene.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch.yml");

    // Convert to Dart
    auto fetch_dart = std::make_shared<darts::Robot>(fetch);
    auto scene_dart = std::make_shared<darts::Structure>("scene", scene);
    auto ground = std::make_shared<darts::Structure>("ground");
    ground->addGround(-0.2);

    // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addStructure(scene_dart);
    world->addStructure(ground);

    // Do a little plan, make a little love, get down tonight
    auto space = std::make_shared<darts::StateSpace>(world);
    space->addGroup(fetch_dart->getName(), GROUP);

    getInitialConfiguration(fetch_dart);
    Eigen::VectorXd start(space->getDimension());
    fetch_dart->getGroupState(GROUP, start);

    getFinalConfiguration(fetch_dart);
    Eigen::VectorXd goal(space->getDimension());
    fetch_dart->getGroupState(GROUP, goal);

    // Create constraint
    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{0.3, 0.8, 0.92});
    Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
    pose.rotate(orn);

    double inf = std::numeric_limits<double>::infinity();
    auto tsr = std::make_shared<darts::TSR>(                                                    //
        fetch_dart, "wrist_roll_link", "base_link",                                             //
        pose,                                                                                   //
        darts::TSR::Bounds{-Eigen::Vector3d{0.5, 0.5, 1e-4}, Eigen::Vector3d{0.5, 0.5, 1e-4}},  //
        darts::TSR::Bounds{-Eigen::Vector3d{1e-4, 1e-4, 1e-4}, Eigen::Vector3d{1e-4, 1e-4, 1e-4}});

    auto tsrc = std::make_shared<darts::TSRConstraint>(space, tsr);
    auto pss = std::make_shared<ompl::base::ProjectedStateSpace>(space, tsrc);
    // auto pss = std::make_shared<ompl::base::AtlasStateSpace>(space, tsrc);

    ompl::geometric::SimpleSetup ss(pss);
    ss.setStateValidityChecker([&](const ompl::base::State *state) {
        auto as = state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState();
        space->setWorldState(world, as);
        return not world->inCollision() and tsrc->isSatisfied(state);
    });

    pss->setSpaceInformation(ss.getSpaceInformation().get());
    pss->setDelta(0.05);
    pss->setLambda(2);

    ompl::base::ScopedState<> start_state(pss);
    start_state.get()
        ->as<ompl::base::ConstrainedStateSpace::StateType>()
        ->getState()
        ->as<darts::StateSpace::StateType>()
        ->data = start;


    ompl::base::ScopedState<> goal_state(pss);
    goal_state.get()
        ->as<ompl::base::ConstrainedStateSpace::StateType>()
        ->getState()
        ->as<darts::StateSpace::StateType>()
        ->data = goal;

    // pss->anchorChart(start_state.get());
    // pss->anchorChart(goal_state.get());

    ss.setStartAndGoalStates(start_state, goal_state);
    auto prm = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.setPlanner(prm);
    // auto rrt = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation(), true);
    // rrt->setRange(1.);
    // ss.setPlanner(rrt);

    std::thread t([&] {
        ompl::base::PlannerStatus solved = ss.solve(10.0);
        auto lock = fetch_dart->getSkeleton()->getLockableReference();

        // Eigen::VectorXd q(fetch_dart->getNumDofsGroup(GROUP));
        // Eigen::VectorXd f(tsr.getDimension());
        // Eigen::MatrixXd j(tsr.getDimension(), fetch_dart->getNumDofsGroup(GROUP));

        // for (unsigned int i = 0; i < 20; ++i)
        // {
        //     fetch_dart->getGroupState(GROUP, q);

        //     tsr.getError(f);
        //     tsr.getJacobian(j);
        //     q -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);

        //     fetch_dart->setGroupState(GROUP, q);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // while (true)
        // {
        // RobotPose pose = RobotPose::Identity();
        // pose.translate(Eigen::Vector3d{x, 0.6, 0.92});
        // Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
        // pose.rotate(orn);

        // tsr.setPose(pose);

        // lock->lock();
        // std::cout << fetch_dart->solveIK() << std::endl;
        // lock->unlock();

        // std::this_thread::sleep_for(std::chrono::milliseconds(30));
        // if (x < 0.45)
        //     x += 0.005;
        // else
        //     x = -0.2;
        // }

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            ss.simplifySolution();

            auto path = ss.getSolutionPath();
            path.interpolate(100);
            path.print(std::cout);

            while (true)
            {
                space->setWorldState(
                    world,
                    path.getStates()[0]->as<ompl::base::ConstrainedStateSpace::StateType>()->getState());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                for (const auto &state : path.getStates())
                {
                    space->setWorldState(
                        world, state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState());
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
