/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ADAPTER_
#define ROBOWFLEX_ADAPTER_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <robowflex_library/macros.h>

namespace robowflex
{
#if ROBOWFLEX_MOVEIT_ISOMETRY
    using RobotPose = Eigen::Isometry3d;
#else
    using RobotPose = Eigen::Affine3d;
#endif
}  // namespace robowflex

#endif
