/* Author: Zachary Kingston */

#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/io/bag.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file ur5_io.cpp
 * Simple test of IO::Bag for loading and saving messages.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Load a scene from a file.
    auto scene = std::make_shared<Scene>(ur5);
    scene->fromYAMLFile("package://robowflex_library/yaml/test.yml");

    // Output scene to a rosbag file.
    {
        IO::Bag bag_out("scene.bag", IO::Bag::WRITE);
        bag_out.addMessage("scene", scene->getMessage());
    }

    // Load the same scene from the rosbag file.
    {
        IO::Bag bag_in("scene.bag", IO::Bag::READ);

        // Load all `moveit_msgs::PlanningScene` from the topic `scene`.
        auto msgs = bag_in.getMessages<moveit_msgs::PlanningScene>({"scene"});
    }

    return 0;
}
