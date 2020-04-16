/* Author: Zachary Kingston */

#include <dart/utils/urdf/urdf.hpp>

#include <robowflex_library/io.h>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>

using namespace robowflex::darts;

static dart::utils::DartLoader urdf_;
static dart::utils::PackageResourceRetriever package_;

void IO::addPackage(const std::string &package)
{
    const auto path = robowflex::IO::resolvePackage("package://" + package);
    if (not path.empty())
        addPackage(package, path);
}

void IO::addPackage(const std::string &package, const std::string &location)
{
    urdf_.addPackageDirectory(package, location);
    package_.addPackageDirectory(package, location);
}

bool IO::loadURDF(Robot &robot, const std::string &urdf)
{
    // Pre-load URDF and extract all relevant ROS packages
    const auto &file = IO::getPackageFile(urdf);
    const auto &text = robowflex::IO::loadXMLToString(file);

    const auto packages = robowflex::IO::findPackageURIs(text);
    for (const auto &package : packages)
        IO::addPackage(package);

    auto skeleton = urdf_.parseSkeleton(urdf);
    if (not skeleton)
        return false;

    skeleton->setSelfCollisionCheck(true);

    for (auto joint : skeleton->getJoints())
        joint->setPositionLimitEnforced(true);

    robot.setSkeleton(skeleton);
    return true;
}

std::string IO::getPackageFile(const std::string &uri)
{
    std::string file = robowflex::IO::resolvePackage(uri);
    if (file.empty())
        file = package_.getFilePath(uri);

    return file;
}
