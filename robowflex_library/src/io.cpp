#include <cstdio>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <array>

#include <boost/filesystem.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <ros/package.h>

#include <eigen_conversions/eigen_msg.h>

#include <robowflex/robowflex.h>

using namespace robowflex;

namespace
{
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

    // is lhs a prefix of rhs?
    bool isPrefix(const std::string &lhs, const std::string &rhs)
    {
        return std::equal(lhs.begin(), lhs.begin() + std::min(lhs.size(), rhs.size()), rhs.begin());
    }

    // is lhs a suffix? of rhs?
    bool isSuffix(const std::string &lhs, const std::string &rhs)
    {
        return std::equal(lhs.rbegin(), lhs.rbegin() + std::min(lhs.size(), rhs.size()), rhs.rbegin());
    }

    bool isXacro(const std::string &path_string)
    {
        boost::filesystem::path path(path_string);
        const std::string extension = boost::filesystem::extension(path);
        return isSuffix("xacro", extension);
    }
}  // namespace

const std::string IO::resolvePath(const std::string &path)
{
    if (path.empty())
        return "";

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

const std::string IO::loadFileToString(const std::string &path)
{
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return "";

    std::ifstream ifs(full_path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(size);
    ifs.read(bytes.data(), size);

    return std::string(bytes.data(), size);
}

const std::string IO::runCommand(const std::string &cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        ROS_ERROR("Failed to run command `%s`!", cmd.c_str());
        return "";
    }

    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }

    return result;
}

const std::string IO::loadXacroToString(const std::string &path)
{
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return "";

    std::string cmd = "rosrun xacro xacro ";

#if ROBOWFLEX_AT_LEAST_MELODIC
#else
    cmd += "--inorder ";
#endif

    cmd += full_path;
    return runCommand(cmd);
}

const std::string IO::loadXMLToString(const std::string &path)
{
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return "";

    if (isXacro(full_path))
        return loadXacroToString(full_path);
    else
        return loadFileToString(full_path);
}

const std::pair<bool, YAML::Node> IO::loadFileToYAML(const std::string &path)
{
    YAML::Node file;
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
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
                    throw Exception(1, "Unknown non-scalar node type in YAML");
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

        throw Exception(1, "Unknown node value in YAML");
    }
}  // namespace

const std::string IO::Handler::UUID(generateUUID());

IO::Handler::Handler(const std::string &name)
  : name_(name), namespace_("robowflex_" + UUID + "/" + name_), nh_(namespace_)
{
}

IO::Handler::~Handler()
{
    for (auto key : params_)
        nh_.deleteParam(key);
}

void IO::Handler::loadYAMLtoROS(const YAML::Node &node, const std::string &prefix)
{
    switch (node.Type())
    {
        case YAML::NodeType::Map:
        {
            const std::string fixed_prefix = (prefix.empty()) ? "" : (prefix + "/");
            for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                loadYAMLtoROS(it->second, fixed_prefix + it->first.as<std::string>());

            break;
        }
        case YAML::NodeType::Sequence:
        case YAML::NodeType::Scalar:
        {
            setParam(prefix, YAMLToXmlRpc(node));
            break;
        }
        default:
            throw Exception(1, "Unknown node type in YAML");
    }
}

const std::string IO::Handler::generateUUID()
{
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();

    std::string s = boost::lexical_cast<std::string>(u);
    std::replace(s.begin(), s.end(), '-', '_');

    return s;
}

std::ofstream IO::createFile(const std::string &file)
{
    boost::filesystem::create_directories(file);
    std::ofstream out(file, std::ofstream::out | std::ofstream::trunc);

    return out;
}
