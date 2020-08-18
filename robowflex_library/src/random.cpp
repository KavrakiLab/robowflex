/* Author: Constantinos Chamzas */

#include <robowflex_library/random.h>

using namespace robowflex;

double random::uniform01()
{
    return uniDist_(generator_);
}

double random::uniformReal(double lower_bound, double upper_bound)
{
    assert(lower_bound <= upper_bound);
    return (upper_bound - lower_bound) * uniform01() + lower_bound;
}

int random::uniformInt(int lower_bound, int upper_bound)
{
    auto r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
    return (r > upper_bound) ? upper_bound : r;
}

bool random::uniformBool()
{
    return uniform01() <= 0.5;
}

double random::gaussian01()
{
    return normalDist_(generator_);
}

double random::gaussian(double mean, double stddev)
{
    return gaussian01() * stddev + mean;
}

double random::gaussian(double stddev)
{
    return gaussian01() * stddev;
}

Eigen::Vector3d random::uniformRPY(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound)
{
    double phi_min = std::max(-pi, lbound[0]);
    double chi_min = std::max(-pi / 2.0, lbound[1]);
    double psi_min = std::max(-pi, lbound[2]);

    double phi_max = std::min(pi, ubound[0]);
    double chi_max = std::min(pi / 2, ubound[1]);
    double psi_max = std::min(pi, ubound[2]);

    Eigen::Vector3d v;
    // From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, 2004
    v[0] = uniformReal(phi_min, phi_max);
    v[1] = acos(uniformReal(cos(chi_min + pi / 2.0), cos(chi_max + pi / 2.0))) - pi / 2.0;
    v[2] = uniformReal(psi_min, psi_max);

    return v;
}

Eigen::Vector3d random::uniformRPY(const Eigen::Vector3d &bounds)
{
    return uniformRPY(-bounds, bounds);
}

Eigen::Vector3d random::unifromRPY()
{
    return uniformRPY(Eigen::Vector3d{-pi, -pi / 2, -pi}, Eigen::Vector3d{pi, -pi / 2, pi});
}

Eigen::Vector3d random::uniformVec(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound)
{
    Eigen::Vector3d vec;
    vec.x() = uniformReal(lbound[0], ubound[0]);
    vec.y() = uniformReal(lbound[1], ubound[1]);
    vec.z() = uniformReal(lbound[2], ubound[2]);

    return vec;
}

Eigen::Vector3d random::uniformVec(const Eigen::Vector3d &bounds)
{
    return uniformVec(-bounds, bounds);
}

Eigen::Vector3d random::gaussianVec(const Eigen::Vector3d &mean, const Eigen::Vector3d &stddev)
{
    Eigen::Vector3d vec;
    vec.x() = gaussian(mean[0], stddev[0]);
    vec.y() = gaussian(mean[1], stddev[1]);
    vec.z() = gaussian(mean[2], stddev[2]);
    return vec;
}

Eigen::Vector3d random::gaussianVec(const Eigen::Vector3d &stddev)
{
    Eigen::Vector3d vec;
    vec.x() = gaussian(stddev[0]);
    vec.y() = gaussian(stddev[1]);
    vec.z() = gaussian(stddev[2]);

    return vec;
}

template <typename Iter>
Iter random::choice(Iter start, Iter end)
{
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(generator_));
    return start;
}

template <typename Type>
Type random::choice(std::vector<Type> vector)
{
    return *choice(vector.begin(), vector.end());
}
