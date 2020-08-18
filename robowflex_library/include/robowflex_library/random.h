/* Author: Constantinos Chamzas */

#ifndef ROBOWFLEX_LIBRARY_RANDOM_
#define ROBOWFLEX_LIBRARY_RANDOM_

#include <random>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robowflex
{
    namespace random
    {
        static std::mt19937 generator_;
        // use the uniform_int_ and uniform real from c++
        static std::uniform_real_distribution<> uniDist_{0, 1};
        static std::normal_distribution<> normalDist_{0, 1};
        static double pi = boost::math::constants::pi<double>();

        double uniform01();

        double uniformReal(double lower_bound, double upper_bound);

        int uniformInt(int lower_bound, int upper_bound);

        bool uniformBool();

        double gaussian01();

        double gaussian(double mean, double stddev);
        double gaussian(double stddev);

        Eigen::Vector3d uniformRPY(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound);

        Eigen::Vector3d uniformRPY(const Eigen::Vector3d &bounds);

        Eigen::Vector3d unifromRPY();

        Eigen::Vector3d uniformVec(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound);

        Eigen::Vector3d uniformVec(const Eigen::Vector3d &bounds);

        Eigen::Vector3d gaussianVec(const Eigen::Vector3d &mean, const Eigen::Vector3d &stddev);

        Eigen::Vector3d gaussianVec(const Eigen::Vector3d &stddev);

        template <typename Iter>
        Iter choice(Iter start, Iter end);

        template <typename Type>
        Type choice(std::vector<Type> vector);
    }  // namespace random

}  // namespace robowflex

#endif
