/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ROSLOG_
#define ROBOWFLEX_ROSLOG_

#include <robowflex_util/log.h>
#include <ros/console.h>

/** \file ROS Logging
 *  All functions follow a 'printf' style, where there is a format string followed by arguments.
 *  e.g., XROS_ERROR("%s failed! Try %+08d", "test", 10)
 *        XROS_ERROR("%1%: %2%", "argument 1", "argument 2")
 *
 *  There are the following debugging levels available, in descending order of priority:
 *  - FATAL, which also causes the program to terminate.
 *  - ERROR
 *  - WARN
 *  - INFO
 *  - DEBUG, which are not visible unless enabled (e.g., with showUpToDebug())
 *
 *  Currently, all logging is done through rosconsole. It is good practice to use the defined
 *  macros here for abstraction purpose.
 */

namespace robowflex
{
    namespace log
    {
        /** \brief Show all logging messages fatal and above.
         */
        void showUpToFatal();

        /** \brief Show all logging messages error and above.
         */
        void showUpToError();

        /** \brief Show all logging messages warning and above.
         */
        void showUpToWarning();

        /** \brief Show all logging messages info and above.
         */
        void showUpToInfo();

        /** \brief Show all logging messages debug and above.
         */
        void showUpToDebug();
    }  // namespace log
}  // namespace robowflex

/**
 * \def XROS_FATAL
 * \brief Output a fatal logging message.
 * \param[in] fmt Format string (see boost::format specification)
 * \param[in] ... Format arguments.
 */
#define XROS_FATAL(fmt, ...) ROS_FATAL_STREAM(robowflex::log::format(fmt, ##__VA_ARGS__).c_str())

/**
 * \def XROS_ERROR
 * \brief Output a error logging message.
 * \param[in] fmt Format string (see boost::format specification)
 * \param[in] ... Format arguments.
 */
#define XROS_ERROR(fmt, ...) ROS_ERROR_STREAM(robowflex::log::format(fmt, ##__VA_ARGS__).c_str())

/**
 * \def XROS_WARN
 * \brief Output a warning logging message.
 * \param[in] fmt Format string (see boost::format specification)
 * \param[in] ... Format arguments.
 */
#define XROS_WARN(fmt, ...) ROS_WARN_STREAM(robowflex::log::format(fmt, ##__VA_ARGS__).c_str())

/**
 * \def XROS_INFO
 * \brief Output a info logging message.
 * \param[in] fmt Format string (see boost::format specification)
 * \param[in] ... Format arguments.
 */
#define XROS_INFO(fmt, ...) ROS_INFO_STREAM(robowflex::log::format(fmt, ##__VA_ARGS__).c_str())

/**
 * \def XROS_DEBUG
 * \brief Output a debug logging message.
 * \param[in] fmt Format string (see boost::format specification)
 * \param[in] ... Format arguments.
 */
#define XROS_DEBUG(fmt, ...) ROS_DEBUG_STREAM(robowflex::log::format(fmt, ##__VA_ARGS__).c_str())

#endif
