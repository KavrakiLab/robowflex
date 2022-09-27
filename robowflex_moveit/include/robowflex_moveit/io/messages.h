/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MOVEIT_MESSAGES_
#define ROBOWFLEX_MOVEIT_MESSAGES_

#include <ros/message_traits.h>  // for message operations

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Compute MD5 hash of message type.
         *  Note that this does not compute a hash over the contents of the message.
         *  \param[in] msg Message of type to hash.
         *  \tparam T Type of the message.
         *  \return The hash of the message type.
         */
        template <typename T>
        std::string getMessageMD5(const T &msg)
        {
            return ros::message_traits::md5sum<T>(msg);
        }
    }  // namespace IO
}  // namespace robowflex

#endif
