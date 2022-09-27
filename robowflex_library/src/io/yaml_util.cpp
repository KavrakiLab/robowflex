/* Author: Zachary Kingston */

#include <robowflex_library/io/filesystem.h>
#include <robowflex_library/io/yaml_utils.h>
#include <robowflex_library/io/yaml_conversions.h>

using namespace robowflex;

bool IO::isNode(const YAML::Node &node)
{
    try
    {
        bool r = node.IsDefined() and not node.IsNull();
        if (r)
            try
            {
                r = node.as<std::string>() != "~";
            }
            catch (std::exception &e)
            {
            }

        return r;
    }
    catch (YAML::InvalidNode &e)
    {
        return false;
    }
}

moveit_msgs::RobotState IO::robotStateFromNode(const YAML::Node &node)
{
    return node.as<moveit_msgs::RobotState>();
}

YAML::Node IO::toNode(const geometry_msgs::Pose &msg)
{
    YAML::Node node;
    node = msg;
    return node;
}

geometry_msgs::Pose IO::poseFromNode(const YAML::Node &node)
{
    return node.as<geometry_msgs::Pose>();
}

YAML::Node IO::toNode(const moveit_msgs::PlanningScene &msg)
{
    YAML::Node node;
    node = msg;
    return node;
}

YAML::Node IO::toNode(const moveit_msgs::MotionPlanRequest &msg)
{
    YAML::Node node;
    node = msg;
    return node;
}

YAML::Node IO::toNode(const moveit_msgs::RobotTrajectory &msg)
{
    YAML::Node node;
    node = msg;
    return node;
}

YAML::Node IO::toNode(const moveit_msgs::RobotState &msg)
{
    YAML::Node node;
    node = msg;
    return node;
}

bool IO::fromYAMLFile(moveit_msgs::PlanningScene &msg, const std::string &file)
{
    return IO::YAMLFileToMessage(msg, file);
}

bool IO::fromYAMLFile(moveit_msgs::MotionPlanRequest &msg, const std::string &file)
{
    return IO::YAMLFileToMessage(msg, file);
}

bool IO::fromYAMLFile(moveit_msgs::RobotState &msg, const std::string &file)
{
    return IO::YAMLFileToMessage(msg, file);
}

bool IO::YAMLToFile(const YAML::Node &node, const std::string &file)
{
    YAML::Emitter out;
    out << node;

    std::ofstream fout;
    IO::createFile(fout, file);

    fout << out.c_str();
    fout.close();

    return true;
}

std::size_t IO::hashYAML(const YAML::Node &node)
{
    std::stringstream sout;
    sout << node;

    return hashString(sout.str());
}

std::pair<bool, YAML::Node> IO::loadFileToYAML(const std::string &path)
{
    YAML::Node file;
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return std::make_pair(false, file);

    if (!IO::isExtension(full_path, "yml") && !IO::isExtension(full_path, "yaml"))
        return std::make_pair(false, file);

    try
    {
        return std::make_pair(true, YAML::LoadFile(full_path));
    }
    catch (std::exception &e)
    {
        return std::make_pair(false, file);
    }
}

std::pair<bool, std::vector<YAML::Node>> IO::loadAllFromFileToYAML(const std::string &path)
{
    std::vector<YAML::Node> file;
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return std::make_pair(false, file);

    if (!IO::isExtension(full_path, "yml") && !IO::isExtension(full_path, "yaml"))
        return std::make_pair(false, file);

    try
    {
        return std::make_pair(true, YAML::LoadAllFromFile(full_path));
    }
    catch (std::exception &e)
    {
        return std::make_pair(false, file);
    }
}
