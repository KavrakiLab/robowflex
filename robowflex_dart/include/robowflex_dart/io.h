/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_IO_
#define ROBOWFLEX_DART_IO_

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(Robot)

        namespace IO
        {
            void addPackage(const std::string &package, const std::string &location);
            bool loadURDF(Robot &robot, const std::string &urdf);
            std::string getPackageFile(const std::string &uri);
        }  // namespace IO
    }      // namespace dart

}  // namespace robowflex

#endif
