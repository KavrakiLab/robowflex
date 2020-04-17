/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_IO_
#define ROBOWFLEX_DART_IO_

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Robot)
        /** \endcond */

        namespace IO
        {
            /** \brief Add a ROS package to Dart's searchable index. Looks up package with rospack.
             *  \param[in] package Package name
             */
            void addPackage(const std::string &package);

            /** \brief Add a ROS package to Dart's searchable index.
             *  \param[in] package Package name
             *  \param[in] location Directory of package
             */
            void addPackage(const std::string &package, const std::string &location);

            /** \brief Loads a URDF at \a urdf into a robot.
             *  \param[out] robot Robot to load URDF in.
             *  \param[in] urdf URDF location.
             *  \return True on success, false on failure.
             */
            bool loadURDF(Robot &robot, const std::string &urdf);

            /** \brief Get the filename for a package URI using Dart's lookup.
             *  \param[in] uri URI to lookup.
             *  \return Path to file.
             */
            std::string getPackageFile(const std::string &uri);
        }  // namespace IO
    }      // namespace darts

}  // namespace robowflex

#endif
