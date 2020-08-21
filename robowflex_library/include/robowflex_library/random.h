/* Author: Constantinos Chamzas */

#ifndef ROBOWFLEX_LIBRARY_RANDOM_
#define ROBOWFLEX_LIBRARY_RANDOM_

#include <random>
#include <robowflex_library/constants.h>

#include <Eigen/Core>

namespace robowflex
{
    /** \brief Collection of methods relating to random sampling
     */
    namespace RND
    {
        static std::mt19937 generator_;
        static std::uniform_real_distribution<> uniDist_{0, 1};
        static std::normal_distribution<> normalDist_{0, 1};

        /** \brief Generate a random real between 0 and 1
         *  \ return sampled number.
         */
        double uniform01();

        /** \brief Generate a random real within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lower_bound Lower bound of uniform distribution.
         *  \param[in] upper_bound Upper bound of uniform distribution.
         *  \return sampled number.
         */
        double uniformReal(double lower_bound, double upper_bound);

        /** \brief Generate a random intiger within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lower_bound Lower bound of uniform distribution.
         *  \param[in] upper_bound Upper bound of uniform distribution.
         *  \return sampled number.
         */
        int uniformInt(int lower_bound, int upper_bound);

        /** \brief Generate a random boolean.
         *  \ return sampled number.
         */
        bool uniformBool();

        /** Generate a random real using a normal distribution with mean 0 and variance 1
         *  \ return sampled number.
         */
        double gaussian01();

        /** \brief Generate a random real using a normal distribution with given \a mean and \e standard
         * deviation
         *  \param[in] mean Mean of the normal distribution.
         *  \param[in] stddev Standard deviation of the normal distribution.
         *  \return sampled number.
         */
        double gaussian(double mean, double stddev);

        /** \brief Generate a random real using a normal distribution with zero mean and given \e standard
         * deviation
         * \param[in] stddev Standard deviation of the normal distribution.
         * \ return sampled number.
         */
        double gaussian(double stddev);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles within lower bound \a lbound and
         * upper bound \a \ubound computed value has the order (roll, pitch, yaw).
         *  \param[in] lbound Lower bound for roll pitch yaw.
         *  \param[in] ubound Upper bound for roll pitch yaw.
         *  \return roll-pitch-yaw vector.
         */
        Eigen::Vector3d uniformRPY(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles within lower bound \a lbound and
         * upper bound \a \ubound computed value has the order (roll, pitch, yaw).
         *  \param[in] bounds [-bounds, bounds] is lower and upper bound is  respectively.
         *  \return roll-pitch-yaw vector.
         */
        Eigen::Vector3d uniformRPY(const Eigen::Vector3d &bounds);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, roll, yaw in range (-pi, pi] and
         * pitch in range(-pi/2, pi/2) computed value has the order (roll, pitch, yaw).
         *  \return roll-pitch-yaw vector.
         */
        Eigen::Vector3d unifromRPY();

        /** \brief Generate a uniform real vector within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lower_bound Lower bound vector of uniform distribution.
         *  \param[in] upper_bound Upper bound vector of uniform distribution.
         * \return sampled number.
         */
        Eigen::Vector3d uniformVec(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound);

        /** \brief Generate a uniform real vector within given bounds: [\a lower_bound, \a upper_bound)
         *  \param[in] lower_bound Lower bound vector of uniform distribution.
         *  \param[in] upper_bound Upper bound vector of uniform distribution.
         * \return sampled number.
         */
        Eigen::Vector3d uniformVec(const Eigen::Vector3d &bounds);

        /** \brief Generate a random real vector using a normal distribution with given \a mean and \e
         * standard deviation
         * \param[in] mean Mean vector of the normal distribution.
         * \param[in] stddev Standard deviation vector (diagonal covariance) of the normal distribution.
         * \return sampled vector.
         */
        Eigen::Vector3d gaussianVec(const Eigen::Vector3d &mean, const Eigen::Vector3d &stddev);

        /** \brief Generate a random real vector using a normal distribution with \e mean zero and \e
         * standard deviation.
         * \param[in] stddev Standard deviation vector (diagonal covariance) of the normal distribution.
         * \return sampled vector.
         */
        Eigen::Vector3d gaussianVec(const Eigen::Vector3d &stddev);

        /** \brief Choose a random element between \a start and \end.
         * \param[in] start Start iterator.
         * \param[in] end End iterator.
         * \return chosen element.
         */
        template <typename Iter>
        Iter choice(Iter start, Iter end);

        /** \brief Choose a random element from a vector.
         * \param[in] start Start iterator.
         * \param[in] end End iterator.
         * \return chosen element.
         */
        template <typename Type>
        Type choice(std::vector<Type> vector);
    }  // namespace RND

}  // namespace robowflex

#endif
