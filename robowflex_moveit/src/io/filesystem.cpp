/* Author: Zachary Kingston */

#include <array>    // for std::array
#include <cstdlib>  // for std::getenv
#include <memory>   // for std::shared_ptr
#include <regex>    // for std::regex
#include <thread>

#include <boost/filesystem.hpp>  // for filesystem paths

#include <robowflex_util/log.h>
#include <robowflex_util/process.h>
#include <robowflex_util/macros.h>
#include <robowflex_moveit/io/filesystem.h>

using namespace robowflex;

namespace
{
    boost::filesystem::path expandHome(const boost::filesystem::path &in)
    {
        const char *home = std::getenv("HOME");
        if (home == nullptr)
            return in;

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
}  // namespace

bool IO::isExtension(const std::string &path_string, const std::string &extension)
{
    boost::filesystem::path path(path_string);
    const std::string last = boost::filesystem::extension(path);
    return isSuffix(extension, last);
}

#include <iostream>
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

        std::string package = IO::runCommand(log::format("rospack find %1%", package_name));

        // Remove new lines.
        package.erase(std::remove(package.begin(), package.end(), '\n'), package.cend());

        if (isPrefix("[rospack] Error", package))
            return "";

        file = package;
        for (auto it = ++subpath.begin(); it != subpath.end(); ++it)
            file /= *it;
    }
    else
        file = path;

    std::cout << expandPath(file).string() << std::endl;
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
        return "";

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
