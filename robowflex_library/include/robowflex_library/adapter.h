/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ADAPTER_
#define ROBOWFLEX_ADAPTER_

#include <type_traits>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <moveit/transforms/transforms.h>

namespace robowflex
{
    /** \brief A pose (point in SE(3)) used in various functions. Defined from what \e MoveIt! uses.
     *
     *  Either an Eigen::Affine3d or Eigen::Isometry3d. These are both transforms, and thus code using a
     *  RobotPose will generally work for both.
     */
    using RobotPose = std::decay<                                      //
        decltype(                                                      //
            std::declval<moveit::core::Transforms>().getTransform("")  //
            )                                                          //
        >::type;

    /** \brief A vector of poses.
     */
    using RobotPoseVector = std::vector<RobotPose, Eigen::aligned_allocator<RobotPose>>;

    /** \brief Convert the Robowflex pose type to another transform type.
     */
    template <typename M>
    M toMatrix(const RobotPose &pose)
    {
        M newpose;
        newpose.matrix() = pose.matrix();
        return newpose;
    }
}  // namespace robowflex

#endif
