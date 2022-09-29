/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MOVEIT_MACROS_
#define ROBOWFLEX_MOVEIT_MACROS_

#include <robowflex_util/macros.h>  // other macros
#include <moveit/version.h>         // for MoveIt version macros

///
/// MoveIt Version Checking
///

/** \def Phrase MoveIt version as integer. */
#define ROBOWFLEX_MOVEIT_VERSION_COMPUTE(major, minor, patch)                                                \
    ((major * 100000u) + (minor * 1000u) + (patch * 1u))
#define ROBOWFLEX_MOVEIT_VERSION                                                                             \
    ROBOWFLEX_MOVEIT_VERSION_COMPUTE(MOVEIT_VERSION_MAJOR, MOVEIT_VERSION_MINOR, MOVEIT_VERSION_PATCH)

#endif
