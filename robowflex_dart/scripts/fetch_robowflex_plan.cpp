#include <thread>
#include <chrono>

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

static const std::string GROUP = "arm";

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

    // // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addStructure(scene_dart);
    world->addStructure(ground);

    // // Do a little plan, make a little love, get down tonight
    auto space = std::make_shared<darts::StateSpace>(world);
    space->addGroup(fetch_dart->getName(), GROUP);

    std::thread t([&] {
        auto lock = fetch_dart->getSkeleton()->getLockableReference();

        darts::TSR tsr(fetch_dart, "wrist_roll_link", "base_link");
        tsr.useGroup(GROUP);

        double x = -0.2;
        RobotPose pose = RobotPose::Identity();
        pose.translate(Eigen::Vector3d{x, 0.6, 0.92});
        Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
        pose.rotate(orn);

        tsr.setPose(pose);

        std::cout << tsr.getDimension() << std::endl;

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
        while (true)
        {
            RobotPose pose = RobotPose::Identity();
            pose.translate(Eigen::Vector3d{x, 0.6, 0.92});
            Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
            pose.rotate(orn);

            tsr.setPose(pose);

            lock->lock();
            std::cout << fetch_dart->solveIK() << std::endl;
            lock->unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            if (x < 0.45)
                x += 0.005;
            else
                x = -0.2;
        }
    });

    world->openOSGViewer();
    return 0;
}
