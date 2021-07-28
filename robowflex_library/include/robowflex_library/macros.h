/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MACROS_
#define ROBOWFLEX_MACROS_

#include <boost/version.hpp>  // for boost version macros

#include <moveit/version.h>  // for MoveIt version macros

#include <ros/common.h>  // for ROS version macros

/** \file */

///
/// Other Macros
///

/** \def Checks if a include file exists. */
#define ROBOWFLEX_INCLUDE_EXISTS(file) __has_include(file)

///
/// ROS Version Checking
///

/** \def Tests if the ROS version is Indigo or higher. */
#define ROBOWFLEX_AT_LEAST_INDIGO ROS_VERSION_MINIMUM(1, 11, 0)

/** \def Tests if the ROS version is Kinetic or higher. */
#define ROBOWFLEX_AT_LEAST_KINETIC ROS_VERSION_MINIMUM(1, 12, 0)

/** \def Tests if the ROS version is Lunar or higher. */
#define ROBOWFLEX_AT_LEAST_LUNAR ROS_VERSION_MINIMUM(1, 13, 0)

/** \def Tests if the ROS version is Melodic or higher. */
#define ROBOWFLEX_AT_LEAST_MELODIC ROS_VERSION_MINIMUM(1, 14, 0)

/** \def Tests if the ROS version is Noetic or higher. */
#define ROBOWFLEX_AT_LEAST_NOETIC ROS_VERSION_MINIMUM(1, 15, 0)

/** \def Tests if the ROS version is not further than Melodic */
#define ROBOWFLEX_AT_MOST_MELODIC                                                                            \
    ROS_VERSION_GE(1, 15, 0, ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH)

/** \def Tests if boost is at least version 1.64 */
#define IS_BOOST_164 (BOOST_VERSION >= 106400u)

/** \def Tests if boost is at least version 1.58 */
#define IS_BOOST_158 (BOOST_VERSION >= 105800u)

#if ROBOWFLEX_AT_LEAST_KINETIC
/** \def Sets flow style for YAML nodes. */
#define ROBOWFLEX_YAML_FLOW(n) n.SetStyle(YAML::EmitterStyle::Flow);
#else
#define ROBOWFLEX_YAML_FLOW(n)
#endif

///
/// MoveIt Version Checking
///

/** \def Phrase MoveIt version as integer. */
#define ROBOWFLEX_MOVEIT_VERSION_COMPUTE(major, minor, patch)                                                \
    ((major * 100000u) + (minor * 1000u) + (patch * 1u))
#define ROBOWFLEX_MOVEIT_VERSION                                                                             \
    ROBOWFLEX_MOVEIT_VERSION_COMPUTE(MOVEIT_VERSION_MAJOR, MOVEIT_VERSION_MINOR, MOVEIT_VERSION_PATCH)

///
/// Compiler Warning Helpers
///

/** \def Stringify helper macro. */
#define ROBOWFLEX_PRAGMA_HELPER0(x) #x
/** \def Pragma generator helper macro. */
#define ROBOWFLEX_PRAGMA_HELPER1(x, y) ROBOWFLEX_PRAGMA_HELPER0(x diagnostic ignored y)

/** \def Push a pragma to disable a compiler warning for GCC. */
#define ROBOWFLEX_PUSH_DISABLE_GCC_WARNING(warning)

/** \def Pop a compiler warning for GCC. */
#define ROBOWFLEX_POP_GCC

/** \def Push a pragma to disable a compiler warning for clang. */
#define ROBOWFLEX_PUSH_DISABLE_CLANG_WARNING(warning)

/** \def Pop a compiler warning for clang. */
#define ROBOWFLEX_POP_CLANG

/** \cond IGNORE */
#if defined(__clang__)
#undef ROBOWFLEX_PUSH_DISABLE_CLANG_WARNING
#undef ROBOWFLEX_POP_CLANG
#define ROBOWFLEX_PUSH_DISABLE_CLANG_WARNING(warning)                                                        \
    _Pragma("clang diagnostic push")                                                                         \
        _Pragma(ROBOWFLEX_PRAGMA_HELPER1(clang, ROBOWFLEX_PRAGMA_HELPER0(warning)))
#define ROBOWFLEX_POP_CLANG _Pragma("GCC diagnostic pop")
#elif defined __GNUC__
#undef ROBOWFLEX_PUSH_DISABLE_GCC_WARNING
#undef ROBOWFLEX_POP_GCC
#define ROBOWFLEX_PUSH_DISABLE_GCC_WARNING(warning)                                                          \
    _Pragma("GCC diagnostic push") _Pragma(ROBOWFLEX_PRAGMA_HELPER1(GCC, ROBOWFLEX_PRAGMA_HELPER0(warning)))
#define ROBOWFLEX_POP_GCC _Pragma("GCC diagnostic pop")
#endif
/** \endcond */

///
/// Type information helpers
///

#if IS_BOOST_158
#include <boost/core/demangle.hpp>
#define ROBOWFLEX_DEMANGLE(x) boost::core::demangle(x)
#else
#include <boost/exception/detail/type_info.hpp>
#define ROBOWFLEX_DEMANGLE(x) boost::units::detail::demangle(x)
#endif

#endif
