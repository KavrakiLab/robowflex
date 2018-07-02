/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_TF_
#define ROBOWFLEX_TF_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <moveit_msgs/BoundingVolume.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /* \endcond */

    /** \brief Collection of methods relating to transforms and transform math.
     */
    namespace TF
    {
        /** \brief Converts a vector message to an Eigen::Vector3d.
         *  \param[in] msg Message to convert.
         *  \return \a msg as an Eigen::Vector3d.
         */
        Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3 &msg);

        /** \brief Converts an Eigen::Vector3d to a vector message.
         *  \param[in] vector Vector to convert.
         *  \return \a vector as a vector message.
         */
        geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d &vector);

        /** \brief Converts a pose message to Eigen::Affine3d.
         *  \param[in] msg Message to convert.
         *  \return \a msg an Eigen::Affine3d.
         */
        Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose &msg);

        /** \brief Converts an Eigen::Affine3d to a pose message.
         *  \param[in] pose Pose to convert.
         *  \return \a pose as a pose message.
         */
        geometry_msgs::Pose poseEigenToMsg(const Eigen::Affine3d &pose);

        /** \brief Converts a quaternion message to Eigen::Quaterniond.
         *  \param[in] msg Message to convert.
         *  \return \a msg an Eigen::Quaterniond.
         */
        Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion &msg);

        /** \brief Converts an Eigen::Quaterniond to a quaternion message.
         *  \param[in] quaternion Quaternion to convert.
         *  \return \a quaternion as a quaternion message.
         */
        geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond &quaternion);

        /** \brief Get a bounding volume message for given \a geometry at a \a pose.
         *  \param[in] pose Pose to place geometry at.
         *  \param[in] geometry Geometry to get bounding volume for.
         *  \return Bounding volume message for \a geometry at \a pose.
         */
        moveit_msgs::BoundingVolume getBoundingVolume(const Eigen::Affine3d &pose, const GeometryConstPtr &geometry);

        /** \brief Get a position constraint message.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         */
        moveit_msgs::PositionConstraint getPositionConstraint(const std::string &ee_name,
                                                              const std::string &base_name,
                                                              const Eigen::Affine3d &pose,
                                                              const GeometryConstPtr &geometry);

        /** \brief Get an orientation constraint message.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string &ee_name,
                                                                    const std::string &base_name,
                                                                    const Eigen::Quaterniond &orientation,
                                                                    const Eigen::Vector3d &tolerances);

        /** \brief Sample an orientation from a given \a orientation with XYZ Euler angle \a tolerances.
         *  \param[in] orientation The desired mean orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        Eigen::Quaterniond sampleOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances);
    }  // namespace TF
}  // namespace robowflex

#endif
