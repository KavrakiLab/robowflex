/* Author: Zachary Kingston */

#include <dart/utils/urdf/urdf.hpp>

#include <se3ez/io.h>
#include <se3ez/robot.h>

using namespace se3ez;

static dart::utils::DartLoader urdf_;
static dart::utils::PackageResourceRetriever package_;

void io::addPackage(const std::string &package, const std::string &location)
{
    urdf_.addPackageDirectory(package, location);
    package_.addPackageDirectory(package, location);
}

void io::loadURDF(Robot &robot, const std::string &urdf)
{
    auto skeleton = urdf_.parseSkeleton(urdf);
    skeleton->setSelfCollisionCheck(true);

    for (auto joint : skeleton->getJoints())
        joint->setPositionLimitEnforced(true);

    robot.setSkeleton(skeleton);
}

std::string io::getPackageFile(const std::string &uri)
{
    return package_.getFilePath(uri);
}
