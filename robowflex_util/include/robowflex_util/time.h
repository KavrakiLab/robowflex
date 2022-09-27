/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_TIME_
#define ROBOWFLEX_TIME_

#include <boost/date_time.hpp>  // for date operations

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Get the current time (up to milliseconds)
         *  \return The time.
         */
        boost::posix_time::ptime getDate();

        /** \brief Get a duration in seconds from two times.
         *  \param[in] start The start time.
         *  \param[in] finish The finish time.
         *  \return The time in seconds.
         */
        double getSeconds(boost::posix_time::ptime start, boost::posix_time::ptime finish);

        /** \brief Put the current thread to sleep for a desired amount of seconds.
         *  \param[in] seconds Seconds to sleep for.
         */
        void threadSleep(double seconds);
    }  // namespace IO
}  // namespace robowflex

#endif
