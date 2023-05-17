/* Author: Zachary Kingston, Constantinos Chamzas */

#ifndef ROBOWFLEX_TF_
#define ROBOWFLEX_TF_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit_msgs/BoundingVolume.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>
#include <robowflex_library/constants.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /* \endcond */

    /** \brief Collection of methods relating to transforms and transform math.
     */
    namespace TF
    {
        /** \brief Creates the Identity pose.
         *  \return A new identity robot pose.
         */
        RobotPose identity();

        /** \brief Creates a robot pose from a linear component and zero orientation.
         *  \param[in] x X-axis translation.
         *  \param[in] y Y-ayis translation.
         *  \param[in] z Z-azis translation.
         *  \return A new robot pose from components.
         */
        RobotPose createPoseXYZ(double x, double y, double z);

        /** \brief Creates a robot pose from a linear component and zero orientation.
         *  \param[in] translation Translation component.
         *  \return A new robot pose from components.
         */
        RobotPose createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d> &translation);

        /** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
         *  \param[in] x X-axis translation.
         *  \param[in] y Y-ayis translation.
         *  \param[in] z Z-azis translation.
         *  \param[in] X Rotation about X.
         *  \param[in] Y Rotation about Y.
         *  \param[in] Z Rotation about Z.
         *  \return A new robot pose from components.
         */
        RobotPose createPoseXYZ(double x, double y, double z, double X, double Y, double Z);

        /** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
         *  \param[in] translation Translation component.
         *  \param[in] rotation Rotational component (X, Y, Z angles).
         *  \return A new robot pose from components.
         */
        RobotPose createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d> &translation,
                                const Eigen::Ref<const Eigen::Vector3d> &rotation);

        /** \brief Creates a robot pose from a linear component and a Quaternion
         *  \param[in] x X-axis translation.
         *  \param[in] y Y-axis translation.
         *  \param[in] z Z-axis translation.
         *  \param[in] W Real quaternion component.
         *  \param[in] X i quaternion component.
         *  \param[in] Y j quaternion component.
         *  \param[in] Z k quaternion component.
         *  \return A new robot pose from components.
         */
        RobotPose createPoseQ(double x, double y, double z, double W, double X, double Y, double Z);

        /** \brief Creates a robot pose from a linear component and a quaternion.
         *  \param[in] translation translation component.
         *  \param[in] rotation rotational component (W, X, Y, Z quaternion values).
         *  \return A new robot pose from components.
         */
        RobotPose createPoseQ(const Eigen::Ref<const Eigen::Vector3d> &translation,
                              const Eigen::Ref<const Eigen::Vector4d> &rotation);

        /** \brief Creates a robot pose from a linear component and a quaternion.
         *  \param[in] translation translation component.
         *  \param[in] rotation rotational component.
         *  \return A new robot pose from components.
         */
        RobotPose createPoseQ(const Eigen::Ref<const Eigen::Vector3d> &translation,
                              const Eigen::Quaterniond &rotation);

        /** \brief Get the rotational component of a robot pose.
         *  \param[in] pose The pose to get the rotation from.
         *  \return The rotational component of the pose.
         */
        Eigen::Quaterniond getPoseRotation(const RobotPose &pose);

        /** \brief Converts a point message to an Eigen::Vector3d.
         *  \param[in] msg Message to convert.
         *  \return \a msg as an Eigen::Vector3d.
         */
        Eigen::Vector3d pointMsgToEigen(const geometry_msgs::Point &msg);

        /** \brief Converts an Eigen::Vector3d to a point message.
         *  \param[in] vector Vector to convert.
         *  \return \a vector as a point message.
         */
        geometry_msgs::Point pointEigenToMsg(const Eigen::Vector3d &vector);

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
         *  \return The position constraint as a MoveIt messsage.
         */
        moveit_msgs::PositionConstraint getPositionConstraint(const std::string &ee_name,
                                                              const std::string &base_name,
                                                              const RobotPose &pose,
                                                              const GeometryConstPtr &geometry);

        Eigen::Vector3d samplePositionConstraint(const moveit_msgs::PositionConstraint &pc);

        /** \brief Get an orientation constraint message.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         *  \return The orientation constraint as a MoveIt message.
         */
        moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string &ee_name,
                                                                    const std::string &base_name,
                                                                    const Eigen::Quaterniond &orientation,
                                                                    const Eigen::Vector3d &tolerances);

        /** \brief Sample an orientation from a given \a orientation with XYZ Euler angle \a tolerances.
         *  \param[in] orientation The desired mean orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         *  \return The sampled orientation.
         */
        Eigen::Quaterniond sampleOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances);

        /** \brief Sample an orientation within the XYZ Euler angle \a tolerances.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         *  \return The sampled orientation.
         */
        Eigen::Quaterniond sampleOrientationUniform(const Eigen::Vector3d &tolerances);

        /** \brief Offset an orientation by a rotation about an axis.
         *  \param[in] orientation Orientation to offset.
         *  \param[in] axis Axis to offset orientation about.
         *  \param[in] value Value by which to offset.
         *  \return The new orientation.
         */
        Eigen::Quaterniond offsetOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &axis, double value);

        /** \brief Sample a position within the given \a bounds using a uniform distribution.
         *  \param[in] bounds The desired mean orientation.
         *  \return The sampled position.
         */
        Eigen::Vector3d samplePositionUniform(const Eigen::Vector3d &bounds);

        /** \brief Sample a position from a gaussian distribution with mean zero and given standard deviation
         *  \param[in] stddev The desired standard deviation for the position.
         *  \return The sampled position.
         */
        Eigen::Vector3d samplePositionGaussian(const Eigen::Vector3d &stddev);

        /** \brief Sample a pose within the given position, orientation bounds.
         *  \param[in] pos_bounds The desired position bounds.
         *  \param[in] orn_bounds The desired orientation bounds.
         *  \return The sampled pose.
         */
        RobotPose samplePoseUniform(const Eigen::Vector3d &pos_bounds, const Eigen::Vector3d &orn_bounds);

        /** \brief Sample a pose with gaussian sampling for the position with given standard deviations and
         *  uniform sampling for the orientation within the given bounds.
         *  \param[in] pos_stddev The desired position standard deviations.
         *  \param[in] orn_bounds The desired orientation bounds.
         *  \return The sampled pose.
         */
        RobotPose samplePoseGaussian(const Eigen::Vector3d &pos_stddev, const Eigen::Vector3d &orn_bounds);

        /** \brief Decode a message as a transform.
         *  \param[in] tf Transform message.
         *  \return The transform.
         */
        RobotPose transformMsgToEigen(const geometry_msgs::TransformStamped &tf);

        /** \brief Encode a transform as a message.
         *  \param[in] source Source frame.
         *  \param[in] target Target frame.
         *  \param[in] tf Transform between frames.
         *  \return Transform message.
         */
        geometry_msgs::TransformStamped transformEigenToMsg(const std::string &source,
                                                            const std::string &target, const RobotPose &tf);

        /** \brief Normalize an angle between -pi to pi.
         *  \param[in] v The angle.
         *  \return The normalized angle.
         */
        double angleNormalize(double v);

        /** \brief Convert an angle to degrees.
         *  \param[in] v The angle in radians.
         *  \return The angle in degrees.
         */
        double toDegrees(double v);

        /** \brief Convert an angle to radians.
         *  \param[in] v The angle in degrees.
         *  \return The angle in radians.
         */
        double toRadians(double v);

        /** \brief Checks if a vector is close to zero.
         *  \param[in] v Vector to check.
         *  \param[in] tolerance Tolerance of what is considered zero.
         *  \return Whether the vector's norm is below the tolerance threshold.
         */
        bool isVecZero(const Eigen::Ref<const Eigen::VectorXd> &v, double tolerance = constants::eps);

    }  // namespace TF
}  // namespace robowflex

#endif
