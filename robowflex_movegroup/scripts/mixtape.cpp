/* Author: Zachary Kingston */

#include <iostream>

#include <ros/console.h>

#include <robowflex_library/util.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/yaml.h>

#include <robowflex_movegroup/services.h>

using namespace robowflex;

movegroup::MoveGroupHelper::Action loadFile(const std::string &filename)
{
    const auto file = IO::loadFileToYAML(filename);
    movegroup::MoveGroupHelper::Action action;

    if (!file.first)
    {
        ROS_DEBUG("Failed to open file %s!", filename.c_str());
        return action;
    }
    else
        ROS_INFO("Opened YAML file %s...", filename.c_str());

    action.id = file.second["id"].as<std::string>();
    action.scene = nullptr;
    action.request = file.second["request"].as<moveit_msgs::MotionPlanRequest>();
    action.success = file.second["success"].as<bool>();
    action.time = file.second["time"].as<double>();

    if (IO::isNode(file.second["trajectory"]))
        action.trajectory = file.second["trajectory"].as<moveit_msgs::RobotTrajectory>();

    return action;
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    auto dirs = ros.getArgs();
    dirs.erase(dirs.begin());  // Erase program name (first arg)

    for (const auto &dir : dirs)
        for (const auto &file : IO::listDirectory(dir).second)
            loadFile(file);
}
