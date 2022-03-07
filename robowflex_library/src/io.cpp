/* Author: Zachary Kingston */

#include <array>    // for std::array
#include <cstdlib>  // for std::getenv
#include <memory>   // for std::shared_ptr
#include <regex>    // for std::regex
#include <thread>

#include <boost/asio/ip/host_name.hpp>                        // for hostname
#include <boost/interprocess/detail/os_thread_functions.hpp>  // for process / thread IDs
#include <boost/filesystem.hpp>                               // for filesystem paths
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // for UUID generation
#include <boost/uuid/uuid_generators.hpp>  // for UUID generation
#include <boost/uuid/uuid_io.hpp>          // for UUID generation
#include <ros/package.h>                   // for package resolving

#include <robowflex_library/io.h>
#include <robowflex_library/io/bag.h>
#include <robowflex_library/io/handler.h>
#include <robowflex_library/log.h>
#include <robowflex_library/macros.h>
#include <robowflex_library/util.h>

using namespace robowflex;

namespace
{
    boost::filesystem::path expandHome(const boost::filesystem::path &in)
    {
        const char *home = std::getenv("HOME");
        if (home == nullptr)
        {
            RBX_WARN("HOME Environment variable is not set! Cannot resolve ~ in path.");
            return in;
        }

        boost::filesystem::path out;
        for (const auto &p : in)
            out /= (p.string() == "~") ? home : p;

        return out;
    }

    boost::filesystem::path expandSymlinks(const boost::filesystem::path &in)
    {
        // Check if the path has a symlink before expansion to avoid error.
        boost::filesystem::path out;
        for (const auto &p : in)
        {
            auto tmp = out / p;
            if (boost::filesystem::is_symlink(tmp))
                return boost::filesystem::canonical(in);
        }

        return in;
    }

    boost::filesystem::path expandPath(const boost::filesystem::path &in)
    {
        boost::filesystem::path out = in;
        out = expandHome(out);
        out = expandSymlinks(out);

        return boost::filesystem::absolute(out);
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

    bool isExtension(const std::string &path_string, const std::string &extension)
    {
        boost::filesystem::path path(path_string);
        const std::string last = boost::filesystem::extension(path);
        return isSuffix(extension, last);
    }
}  // namespace

std::string IO::resolvePackage(const std::string &path)
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
            RBX_WARN("Package `%s` does not exist.", package_name);
            return "";
        }

        file = package;
        for (auto it = ++subpath.begin(); it != subpath.end(); ++it)
            file /= *it;
    }
    else
        file = path;

    return expandPath(file).string();
}

std::set<std::string> IO::findPackageURIs(const std::string &string)
{
    const std::regex re(R"(((package):?\/)\/?([^:\/\s]+)((\/\w+)*\/)([\w\-\.]+[^#?\s]+)?)");

    std::set<std::string> packages;

    auto begin = std::sregex_iterator(string.begin(), string.end(), re);
    auto end = std::sregex_iterator();

    for (auto it = begin; it != end; ++it)
    {
        std::smatch sm = *it;
        std::string smstr = sm.str(3);
        packages.emplace(smstr);
    }

    return packages;
}

std::string IO::resolvePath(const std::string &path)
{
    boost::filesystem::path file = resolvePackage(path);

    if (!boost::filesystem::exists(file))
    {
        RBX_WARN("File `%s` does not exist.", path);
        return "";
    }

    return boost::filesystem::canonical(boost::filesystem::absolute(file)).string();
}

std::string IO::resolveParent(const std::string &path)
{
    boost::filesystem::path file = resolvePackage(path);
    return file.parent_path().string();
}

std::string IO::makeFilepath(const std::string &directory, const std::string &filename)
{
    boost::filesystem::path dirpath = resolveParent(directory);
    dirpath /= filename;

    return dirpath.string();
}

std::string IO::loadFileToString(const std::string &path)
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

std::string IO::runCommand(const std::string &cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        RBX_ERROR("Failed to run command `%s`!", cmd);
        return "";
    }

    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }

    return result;
}

std::string IO::loadXacroToString(const std::string &path)
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

std::string IO::loadXMLToString(const std::string &path)
{
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return "";

    if (isExtension(full_path, "xacro"))
        return loadXacroToString(full_path);

    return loadFileToString(full_path);
}

std::pair<bool, YAML::Node> IO::loadFileToYAML(const std::string &path)
{
    YAML::Node file;
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return std::make_pair(false, file);

    if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
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

    if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
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

std::string IO::generateUUID()
{
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();

    std::string s = boost::lexical_cast<std::string>(u);

    std::replace(s.begin(), s.end(), '-', '_');

    return s;
}

void IO::createFile(std::ofstream &out, const std::string &file)
{
    boost::filesystem::path path(file);
    path = expandHome(path);
    path = expandSymlinks(path);

    const auto parent = path.parent_path().string();

    if (!parent.empty())
        boost::filesystem::create_directories(parent);

    out.open(path.string(), std::ofstream::out | std::ofstream::trunc);
}

std::string IO::createTempFile(std::ofstream &out)
{
    auto temp = boost::filesystem::unique_path();
    auto filename = "/tmp/" + temp.string();
    createFile(out, filename);

    return filename;
}

void IO::deleteFile(const std::string &file)
{
    boost::filesystem::path path(file);
    path = expandHome(path);
    path = expandSymlinks(path);

    boost::filesystem::remove(path);
}

std::pair<bool, std::vector<std::string>> IO::listDirectory(const std::string &directory)
{
    std::vector<std::string> contents;

    const std::string full_path = resolvePath(directory);
    if (full_path.empty())
        return std::make_pair(false, contents);

    boost::filesystem::path path(full_path);
    if (!boost::filesystem::is_directory(path))
        return std::make_pair(false, contents);

    for (auto it = boost::filesystem::directory_iterator(path); it != boost::filesystem::directory_iterator();
         ++it)
        contents.emplace_back(expandPath(it->path()).string());

    return std::make_pair(true, contents);
}

std::string IO::getHostname()
{
    return boost::asio::ip::host_name();
}

std::size_t IO::getProcessID()
{
    return boost::interprocess::ipcdetail::get_current_process_id();
}

std::size_t IO::getThreadID()
{
    return boost::interprocess::ipcdetail::get_current_thread_id();
}

boost::posix_time::ptime IO::getDate()
{
    return boost::posix_time::microsec_clock::local_time();
}

double IO::getSeconds(boost::posix_time::ptime start, boost::posix_time::ptime finish)
{
    auto duration = finish - start;
    return duration.total_microseconds() / 1000000.;
}

void IO::threadSleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long int>(seconds * 1000)));
}

template <typename T>
std::vector<T> IO::tokenize(const std::string &s, const std::string &separators)
{
    boost::char_separator<char> seps(separators.c_str());
    boost::tokenizer<boost::char_separator<char>> tokenizer(s, seps);

    std::vector<T> values;
    std::transform(tokenizer.begin(), tokenizer.end(), std::back_inserter(values),
                   [](const std::string &s) { return boost::lexical_cast<T>(s); });

    return std::vector<T>();
}

template std::vector<std::string> IO::tokenize(const std::string &, const std::string &);
template std::vector<double> IO::tokenize(const std::string &, const std::string &);

///
/// IO::Bag
///

IO::Bag::Bag(const std::string &file, Mode mode)
  : mode_(mode)
  , file_((mode_ == WRITE) ? file : IO::resolvePath(file))
  , bag_(file_, (mode_ == WRITE) ? rosbag::bagmode::Write : rosbag::bagmode::Read)
{
}

IO::Bag::~Bag()
{
    bag_.close();
}

///
/// IO::Handler
///

// Static ID for all handlers
const std::string IO::Handler::UUID(generateUUID());

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

IO::Handler::Handler(const std::string &name)
  : name_(name), namespace_("robowflex_" + UUID + "/" + name_), nh_(namespace_)
{
}

IO::Handler::Handler(const IO::Handler &handler, const std::string &name)
  : name_(handler.getName()), namespace_(handler.getNamespace()), nh_(handler.getHandle(), name)
{
}

IO::Handler::~Handler()
{
    for (const auto &key : params_)
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

bool IO::Handler::hasParam(const std::string &key) const
{
    return nh_.hasParam(key);
}

const ros::NodeHandle &IO::Handler::getHandle() const
{
    return nh_;
}

const std::string &IO::Handler::getName() const
{
    return name_;
}

const std::string &IO::Handler::getNamespace() const
{
    return namespace_;
}
