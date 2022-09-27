/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_UID_
#define ROBOWFLEX_UID_

#include <cstddef>
#include <string>

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Generates a UUID.
         *  \return String of UUID.
         */
        std::string generateUUID();

        /** \brief Compute hash of string.
         *  \param[in] string String to hash.
         *  \return A hashed value of the string. Note: not cryptographically secure.
         */
        std::size_t hashString(const std::string &string);
    }  // namespace IO
}  // namespace robowflex

#endif
