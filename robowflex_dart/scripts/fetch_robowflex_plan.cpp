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

static const std::string GROUP = "arm_with_torso";

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
        double x = -0.2;

        darts::TSR tsr("wrist_roll_link", "base_link");
        while (true)
        {
            RobotPose pose = RobotPose::Identity();
            pose.translate(Eigen::Vector3d{x, 0.6, 0.92});
            Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
            pose.rotate(orn);

            tsr.setPose(pose);
            tsr.setIKTarget(fetch_dart);

            fetch_dart->solveIK();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            x += 0.001;
        }
    });

    world->openOSGViewer();
    return 0;
}
