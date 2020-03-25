#include <robowflex_library/util.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>

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

    // // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addStructure(scene_dart);

    // // Do a little plan, make a little love, get down tonight
    // auto space = std::make_shared<darts::StateSpace>(world);
    // space->addGroup(fetch_dart->getName(), "arm_with_torso");

    world->openOSGViewer();
    return 0;
}
