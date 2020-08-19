/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_CONSTANTS_
#define ROBOWFLEX_CONSTANTS_

#include <boost/math/constants/constants.hpp>

namespace robowflex
{
    namespace constants
    {
        // common
        const double half = boost::math::constants::half<double>();
        const double third = boost::math::constants::third<double>();
        const double eps = 1e-8;

        // pi
        const double pi = boost::math::constants::pi<double>();
        const double half_pi = boost::math::constants::half_pi<double>();
        const double quarter_pi = half_pi * half;
        const double two_pi = boost::math::constants::two_pi<double>();
    }  // namespace constants
}  // namespace robowflex

#endif
