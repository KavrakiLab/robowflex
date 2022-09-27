/* Author: Constantinos Chamzas, Zachary Kingston */

#ifndef ROBOWFLEX_RANDOM_
#define ROBOWFLEX_RANDOM_

#include <random>
#include <robowflex_util/constants.h>
#include <robowflex_util/math.h>

namespace robowflex
{
    /** \brief Collection of methods relating to random sampling
     */
    namespace RNG
    {
        /** \brief Set the random seed of the underlying generator.
         *  \param[in] seed Seed to set in generator.
         */
        void setSeed(unsigned int seed);

        /** \brief Generate a random real in  [0,1).
         *  \return Sampled number.
         */
        double uniform01();

        /** \brief Generate a random real within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lower_bound Lower bound of uniform distribution.
         *  \param[in] upper_bound Upper bound of uniform distribution.
         *  \return Sampled number.
         */
        double uniformReal(double lower_bound, double upper_bound);

        /** \brief Generate a random integer within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lower_bound Lower bound of uniform distribution.
         *  \param[in] upper_bound Upper bound of uniform distribution.
         *  \return Sampled number.
         */
        int uniformInt(int lower_bound, int upper_bound);

        /** \brief Generate a random boolean.
         *  \return Sampled number.
         */
        bool uniformBool();

        /** Generate a random real using a normal distribution with mean 0 and variance 1
         *  \return Sampled number.
         */
        double gaussian01();

        /** \brief Generate a random real using a normal distribution with given \a mean and \e standard
         * deviation.
         *  \param[in] mean Mean of the normal distribution.
         *  \param[in] stddev Standard deviation of the normal distribution.
         *  \return Sampled number.
         */
        double gaussian(double mean, double stddev);

        /** \brief Generate a random real using a normal distribution with zero mean and given \e standard
         * deviation.
         *  \param[in] stddev Standard deviation of the normal distribution.
         *  \return Sampled number.
         */
        double gaussian(double stddev);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles within lower bound \a lbound and
         * upper bound \a ubound computed value has the order (roll, pitch, yaw).
         *  \param[in] lbound Lower bound for roll pitch yaw.
         *  \param[in] ubound Upper bound for roll pitch yaw.
         *  \return roll-pitch-yaw vector.
         */
        Eigen::Vector3d uniformRPY(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles within lower bound \a lbound and
         * upper bound \a ubound computed value has the order (roll, pitch, yaw).
         *  \param[in] bounds [-bounds, bounds] is lower and upper bound is  respectively.
         *  \return roll-pitch-yaw vector.
         */
        Eigen::Vector3d uniformRPY(const Eigen::Vector3d &bounds);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, roll, yaw in range [-pi, pi) and
         * pitch in range[-pi/2, pi/2) computed value has the order (roll, pitch, yaw).
         *  \return roll-pitch-yaw vector.
         */
        Eigen::Vector3d unifromRPY();

        /** \brief Generate a uniform real vector within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lbound Lower bound vector of uniform distribution.
         *  \param[in] ubound Upper bound vector of uniform distribution.
         *  \return Sampled vector.
         */
        Eigen::Vector3d uniformVec(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound);

        /** \brief Generate a uniform real vector within given bounds: [\a -bounds, \a bounds)
         *  \param[in] bounds Upper and (negative) lower bound vector of uniform distribution.
         *  \return Sampled vector.
         */
        Eigen::Vector3d uniformVec(const Eigen::Vector3d &bounds);

        /** \brief Generate a random real vector using a normal distribution with given \a mean and \e
         * standard deviation
         *  \param[in] mean Mean vector of the normal distribution.
         *  \param[in] stddev Standard deviation vector (diagonal covariance) of the normal distribution.
         *  \return Sampled vector.
         */
        Eigen::Vector3d gaussianVec(const Eigen::Vector3d &mean, const Eigen::Vector3d &stddev);

        /** \brief Generate a random real vector using a normal distribution with \e mean zero and \e
         * standard deviation.
         *  \param[in] stddev Standard deviation vector (diagonal covariance) of the normal distribution.
         *  \return Sampled vector.
         */
        Eigen::Vector3d gaussianVec(const Eigen::Vector3d &stddev);

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

        /** \brief Choose a random element between \a start and \a end.
         *  \param[in] start Start iterator.
         *  \param[in] end End iterator.
         *  \return Chosen element.
         */
        template <typename Iter>
        Iter uniformSample(Iter start, Iter end)
        {
            std::advance(start, uniformInt(0, std::distance(start, end)));
            return start;
        };

        /** \brief Choose a random element from a vector.
         *  \param[in] vector Vector to sample from.
         *  \return Chosen element.
         */
        template <typename Type>
        Type &uniformSample(std::vector<Type> &vector)
        {
            return *uniformSample(vector.begin(), vector.end());
        }
    }  // namespace RNG

}  // namespace robowflex

#endif
