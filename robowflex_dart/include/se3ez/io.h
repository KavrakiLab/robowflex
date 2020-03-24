/* Author: Zachary Kingston */

#ifndef SE3EZ_IO_
#define SE3EZ_IO_

#include <se3ez/util.h>

namespace se3ez
{
    SE3EZ_CLASS(Robot)

    namespace io
    {
        void addPackage(const std::string &package, const std::string &location);
        void loadURDF(Robot &robot, const std::string &urdf);
        std::string getPackageFile(const std::string &uri);
    }  // namespace io
}  // namespace se3ez

#endif
