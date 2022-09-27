/* Author: Zachary Kingston, Constantinos Chamzas */

#ifndef ROBOWFLEX_MOVEIT_TF_
#define ROBOWFLEX_MOVEIT_TF_

#include <robowflex_util/class_forward.h>
#include <robowflex_moveit/utility/adapter.h>
#include <robowflex_util/constants.h>

namespace robowflex
{
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

        /** \brief Sample an orientation from a given \a orientation with XYZ Euler angle \a tolerances.
         *  \param[in] orientation The desired mean orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         *  \return The sampled orientation.
         */
        Eigen::Quaterniond sampleOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances);

        /** \brief Sample an orientation within the XYZ Euler angle \a bounds.
         *  \param[in] bounds XYZ Euler angle bounds about orientation.
         *  \return The sampled orientation.
         */
        Eigen::Quaterniond sampleOrientationUniform(const Eigen::Vector3d &bounds);

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

        /** \brief Sample a pose with gaussian sampling for the position with given variances and
         *  uniform sampling for the orientation within the given bounds.
         *  \param[in] pos_variances The desired position variances.
         *  \param[in] orn_bounds The desired orientation bounds.
         *  \return The sampled pose.
         */
        RobotPose samplePoseGaussian(const Eigen::Vector3d &pos_variances, const Eigen::Vector3d &orn_bounds);

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

        /** \brief Split a string into a vector of doubles.
         *  \param[in] s String to separate.
         *  \param[in] separators Separators to split string on.
         *  \return The vector of doubles contained in the string.
         */
        std::vector<double> stringToVec(const std::string &s, const std::string &separators = " ");
    }  // namespace TF
}  // namespace robowflex

#endif
