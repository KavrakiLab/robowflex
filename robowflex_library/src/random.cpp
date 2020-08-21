/* Author: Constantinos Chamzas */

#include <robowflex_library/random.h>
#include <robowflex_library/constants.h>

using namespace robowflex;

double RND::uniform01()
{
    return uniDist_(generator_);
}

double RND::uniformReal(double lower_bound, double upper_bound)
{
    assert(lower_bound <= upper_bound);
    return (upper_bound - lower_bound) * uniform01() + lower_bound;
}

int RND::uniformInt(int lower_bound, int upper_bound)
{
    auto r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
    return (r > upper_bound) ? upper_bound : r;
}

bool RND::uniformBool()
{
    return uniform01() <= 0.5;
}

double RND::gaussian01()
{
    return normalDist_(generator_);
}

double RND::gaussian(double mean, double stddev)
{
    return gaussian01() * stddev + mean;
}

double RND::gaussian(double stddev)
{
    return gaussian01() * stddev;
}

Eigen::Vector3d RND::uniformRPY(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound)
{
    const double &pi = constants::pi;
    const double &half_pi = constants::half_pi;
    double phi_min = std::max(-pi, lbound[0]);
    double chi_min = std::max(-half_pi, lbound[1]);
    double psi_min = std::max(-pi, lbound[2]);

    double phi_max = std::min(pi, ubound[0]);
    double chi_max = std::min(half_pi, ubound[1]);
    double psi_max = std::min(pi, ubound[2]);

    Eigen::Vector3d v;
    // From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, 2004
    v[0] = uniformReal(phi_min, phi_max);
    v[1] = acos(uniformReal(cos(chi_min + half_pi), cos(chi_max + half_pi))) - half_pi;
    v[2] = uniformReal(psi_min, psi_max);

    return v;
}

Eigen::Vector3d RND::uniformRPY(const Eigen::Vector3d &bounds)
{
    return uniformRPY(-bounds, bounds);
}

Eigen::Vector3d RND::unifromRPY()
{
    const double &pi = constants::pi;
    const double &half_pi = constants::half_pi;
    return uniformRPY(Eigen::Vector3d{-pi, -half_pi, -pi}, Eigen::Vector3d{pi, half_pi, pi});
}

Eigen::Vector3d RND::uniformVec(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound)
{
    Eigen::Vector3d vec;
    vec.x() = uniformReal(lbound[0], ubound[0]);
    vec.y() = uniformReal(lbound[1], ubound[1]);
    vec.z() = uniformReal(lbound[2], ubound[2]);

    return vec;
}

Eigen::Vector3d RND::uniformVec(const Eigen::Vector3d &bounds)
{
    return uniformVec(-bounds, bounds);
}

Eigen::Vector3d RND::gaussianVec(const Eigen::Vector3d &mean, const Eigen::Vector3d &stddev)
{
    Eigen::Vector3d vec;
    vec.x() = gaussian(mean[0], stddev[0]);
    vec.y() = gaussian(mean[1], stddev[1]);
    vec.z() = gaussian(mean[2], stddev[2]);
    return vec;
}

Eigen::Vector3d RND::gaussianVec(const Eigen::Vector3d &stddev)
{
    Eigen::Vector3d vec;
    vec.x() = gaussian(stddev[0]);
    vec.y() = gaussian(stddev[1]);
    vec.z() = gaussian(stddev[2]);

    return vec;
}

template <typename Iter>
Iter RND::choice(Iter start, Iter end)
{
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(generator_));
    return start;
}

template <typename Type>
Type RND::choice(std::vector<Type> vector)
{
    return *choice(vector.begin(), vector.end());
}
