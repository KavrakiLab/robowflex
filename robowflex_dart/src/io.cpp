/* Author: Zachary Kingston */

#include <dart/utils/urdf/urdf.hpp>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>

using namespace robowflex::darts;

static dart::utils::DartLoader urdf_;
static dart::utils::PackageResourceRetriever package_;

void IO::addPackage(const std::string &package, const std::string &location)
{
    urdf_.addPackageDirectory(package, location);
    package_.addPackageDirectory(package, location);
}

void IO::loadURDF(Robot &robot, const std::string &urdf)
{
    auto skeleton = urdf_.parseSkeleton(urdf);
    skeleton->setSelfCollisionCheck(true);

    for (auto joint : skeleton->getJoints())
        joint->setPositionLimitEnforced(true);

    robot.setSkeleton(skeleton);
}

std::string IO::getPackageFile(const std::string &uri)
{
    return package_.getFilePath(uri);
}
