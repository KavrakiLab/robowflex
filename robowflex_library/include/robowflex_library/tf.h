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
#include <robowflex_library/adapter.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /* \endcond */

    /** \brief Collection of methods relating to transforms and transform math.
     */
    namespace TF
    {
        /** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
         *  \param[in] x X-axis translation
         *  \param[in] y Y-ayis translation
         *  \param[in] z Z-azis translation
         *  \param[in] X Rotation about X
         *  \param[in] Y Rotation about Y
         *  \param[in] Z Rotation about Z
         *  \return A new robot pose from components.
         */
        RobotPose createPoseXYZ(double x, double y, double z, double X, double Y, double Z);

        /** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
         *  \param[in] translation translation component
         *  \param[in] rotation rotational component (X, Y, Z angles)
         *  \return A new robot pose from components.
         */
        RobotPose createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d> &translation, const Eigen::Ref<const Eigen::Vector3d> &rotation);

        /** \brief Creates a robot pose from a linear component and a Quaternion
         *  \param[in] x X-axis translation
         *  \param[in] y Y-axis translation
         *  \param[in] z Z-axis translation
         *  \param[in] W Real quaternion component.
         *  \param[in] X i quaternion component.
         *  \param[in] Y j quaternion component.
         *  \param[in] Z k quaternion component.
         *  \return A new robot pose from components.
         */
        RobotPose createPoseQ(double x, double y, double z, double W, double X, double Y, double Z);

        /** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
         *  \param[in] translation translation component
         *  \param[in] rotation rotational component (W, X, Y, Z quaternion values)
         *  \return A new robot pose from components.
         */
        RobotPose createPoseQ(const Eigen::Ref<const Eigen::Vector3d> &translation, const Eigen::Ref<const Eigen::Vector4d> &rotation);

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

        /** \brief Converts a pose message to RobotPose.
         *  \param[in] msg Message to convert.
         *  \return \a msg an RobotPose.
         */
        RobotPose poseMsgToEigen(const geometry_msgs::Pose &msg);

        /** \brief Converts an RobotPose to a pose message.
         *  \param[in] pose Pose to convert.
         *  \return \a pose as a pose message.
         */
        geometry_msgs::Pose poseEigenToMsg(const RobotPose &pose);

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
        moveit_msgs::BoundingVolume getBoundingVolume(const RobotPose &pose,
                                                      const GeometryConstPtr &geometry);

        /** \brief Get a position constraint message.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         */
        moveit_msgs::PositionConstraint getPositionConstraint(const std::string &ee_name,
                                                              const std::string &base_name,
                                                              const RobotPose &pose,
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

        /** \brief Offset an orientation by a rotation about an axis.
         *  \param[in] orientation Orientation to offset.
         *  \param[in] axis Axis to offset orientation about.
         *  \param[in] value Value by which to offset.
         */
        Eigen::Quaterniond offsetOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &axis, double value);
    }  // namespace TF
}  // namespace robowflex

#endif
