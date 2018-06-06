#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <moveit/rdf_loader/rdf_loader.h>

#include "robowflex.h"

boost::filesystem::path expandHome(const boost::filesystem::path &in)
{
    const char *home = getenv("HOME");
    if (home == NULL)
    {
        ROS_WARN("HOME Environment variable is not set! Cannot resolve ~ in path.");
        return in;
    }

    boost::filesystem::path out;
    for (auto p : in)
        out /= (p.string() == "~") ? home : p;

    return out;
}

boost::filesystem::path expandSymlinks(const boost::filesystem::path &in)
{
    boost::filesystem::path out;
    for (auto p : in)
    {
        auto tmp = out / p;
        if (boost::filesystem::is_symlink(tmp))
            out = boost::filesystem::read_symlink(tmp);
        else
            out /= p;
    }

    return out;
}

bool isPrefix(const std::string &lhs, const std::string &rhs)
{
    return std::equal(lhs.begin(), lhs.begin() + std::min(lhs.size(), rhs.size()), rhs.begin());
}

const std::string robowflex::resolvePath(const std::string &path)
{
    const std::string prefix = "package://";

    boost::filesystem::path file;
    if (isPrefix(prefix, path))
    {
        boost::filesystem::path subpath(path.substr(prefix.length(), path.length() - 1));
        const std::string package_name = (*subpath.begin()).string();

        const std::string package = ros::package::getPath(package_name);
        if (package.empty())
        {
            ROS_WARN("Package `%s` does not exist.", package_name.c_str());
            return "";
        }

        file = package;
        for (auto it = ++subpath.begin(); it != subpath.end(); ++it)
            file /= *it;
    }
    else
        file = path;

    file = expandHome(file);
    file = expandSymlinks(file);

    if (!boost::filesystem::exists(file))
    {
        ROS_WARN("File `%s` does not exist.", file.string().c_str());
        return "";
    }

    return boost::filesystem::canonical(boost::filesystem::absolute(file)).string();
}

const std::string robowflex::loadFileToXML(const std::string &path)
{
    const std::string full_path = robowflex::resolvePath(path);
    if (full_path.empty())
        return "";

    std::string buffer;
    if (!rdf_loader::RDFLoader::loadXmlFileToString(buffer, full_path, {}))
    {
        ROS_ERROR("Failed to load file `%s` to XML", path.c_str());
        return "";
    }

    return buffer;
}

const YAML::Node robowflex::loadFileToYAML(const std::string &path)
{
    YAML::Node file;
    const std::string full_path = robowflex::resolvePath(path);
    if (full_path.empty())
        return file;

    return YAML::LoadFile(full_path);
}

namespace
{
    class XmlRpcValueCreator : public XmlRpc::XmlRpcValue
    {
    public:
        static XmlRpcValueCreator createArray(const std::vector<XmlRpcValue> &values)
        {
            XmlRpcValueCreator ret;
            ret._type = TypeArray;
            ret._value.asArray = new ValueArray(values);

            return ret;
        }

        static XmlRpcValueCreator createStruct(const std::map<std::string, XmlRpcValue> &members)
        {
            XmlRpcValueCreator ret;
            ret._type = TypeStruct;
            ret._value.asStruct = new std::map<std::string, XmlRpcValue>(members);
            return ret;
        }
    };

    XmlRpc::XmlRpcValue YAMLToXmlRpc(const YAML::Node &node)
    {
        if (node.Type() != YAML::NodeType::Scalar)
        {
            switch (node.Type())
            {
                case YAML::NodeType::Map:
                {
                    std::map<std::string, XmlRpc::XmlRpcValue> members;
                    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                        members[it->first.as<std::string>()] = YAMLToXmlRpc(it->second);

                    return XmlRpcValueCreator::createStruct(members);
                }
                case YAML::NodeType::Sequence:
                {
                    std::vector<XmlRpc::XmlRpcValue> values;
                    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                        values.push_back(YAMLToXmlRpc(*it));

                    return XmlRpcValueCreator::createArray(values);
                }
                default:
                {
                    // TODO : Throw
                }
            }
        }

        if (node.Tag() == "!!int")
            return XmlRpc::XmlRpcValue(node.as<int>());

        if (node.Tag() == "!!float")
            return XmlRpc::XmlRpcValue(node.as<double>());

        if (node.Tag() == "!!bool")
            return XmlRpc::XmlRpcValue(node.as<bool>());

        try
        {
            return XmlRpc::XmlRpcValue(node.as<bool>());
        }
        catch (YAML::Exception &)
        {
        }

        try
        {
            return XmlRpc::XmlRpcValue(node.as<int>());
        }
        catch (YAML::Exception &)
        {
        }

        try
        {
            return XmlRpc::XmlRpcValue(node.as<double>());
        }
        catch (YAML::Exception &)
        {
        }

        try
        {
            return XmlRpc::XmlRpcValue(node.as<std::string>());
        }
        catch (YAML::Exception &)
        {
        }

        // TODO : Throw
    }
}  // namespace

void robowflex::loadYAMLParams(const YAML::Node &node, const std::string &prefix)
{
    switch (node.Type())
    {
        case YAML::NodeType::Map:
        {
            for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                loadYAMLParams(it->second, prefix + "/" + it->first.as<std::string>());

            break;
        }
        case YAML::NodeType::Sequence:
        case YAML::NodeType::Scalar:
        {
            ros::param::set(prefix, YAMLToXmlRpc(node));
            break;
        }
        default:
        {
            // TODO: throw
        }
    }
}
