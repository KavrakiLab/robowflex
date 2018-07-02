/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MACROS_
#define ROBOWFLEX_MACROS_

#include <boost/version.hpp>  // for boost version macros

#include <ros/common.h>  // for ROS version macros

/** \file */

/** \brief Tests if the ROS version is Indigo or higher. */
#define ROBOWFLEX_AT_LEAST_INDIGO ROS_VERSION_MINIMUM(1, 11, 0)

/** \brief Tests if the ROS version is Lunar or higher. */
#define ROBOWFLEX_AT_LEAST_LUNAR ROS_VERSION_MINIMUM(1, 12, 0)

/** \brief Tests if the ROS version is Kinetic or higher. */
#define ROBOWFLEX_AT_LEAST_KINETIC ROS_VERSION_MINIMUM(1, 13, 0)

/** \brief Tests if the ROS version is Melodic or higher. */
#define ROBOWFLEX_AT_LEAST_MELODIC ROS_VERSION_MINIMUM(1, 14, 0)

/** \brief Tests if boost is at least version 1.64 */
#define IS_BOOST_164 BOOST_VERSION >= 106400

#endif
