/* Author: Constantinos Chamzas */

#include <robowflex_library/constants.h>
#include <robowflex_library/random.h>

using namespace robowflex;

namespace
{
    static std::mt19937 GENERATOR;                          ///< Random engine generator.
    static std::uniform_real_distribution<> UNIDIST{0, 1};  ///< Uniform distribution.
    static std::normal_distribution<> NORMALDIST{0, 1};     ///< Normal distribution.
}  // namespace

void RNG::setSeed(unsigned int seed)
{
    GENERATOR.seed(seed);
}

double RNG::uniform01()
{
    return UNIDIST(GENERATOR);
}

double RNG::uniformReal(double lower_bound, double upper_bound)
{
    assert(lower_bound <= upper_bound);
    return (upper_bound - lower_bound) * uniform01() + lower_bound;
}

int RNG::uniformInt(int lower_bound, int upper_bound)
{
    auto r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound)));
    return (r > upper_bound - 1) ? upper_bound - 1 : r;
}

bool RNG::uniformBool()
{
    return uniform01() <= 0.5;
}

double RNG::gaussian01()
{
    return NORMALDIST(GENERATOR);
}

double RNG::gaussian(double mean, double stddev)
{
    return gaussian01() * stddev + mean;
}

double RNG::gaussian(double stddev)
{
    return gaussian01() * stddev;
}

Eigen::Vector3d RNG::uniformRPY(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound)
{
    const double phi_min = std::max(-constants::pi, lbound[0]);
    const double chi_min = std::max(-constants::half_pi, lbound[1]);
    const double psi_min = std::max(-constants::pi, lbound[2]);

    const double phi_max = std::min(constants::pi, ubound[0]);
    const double chi_max = std::min(constants::half_pi, ubound[1]);
    const double psi_max = std::min(constants::pi, ubound[2]);

    Eigen::Vector3d v;
    // From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, 2004
    v[0] = uniformReal(phi_min, phi_max);
    v[1] = acos(uniformReal(cos(chi_min + constants::half_pi), cos(chi_max + constants::half_pi))) -
           constants::half_pi;
    v[2] = uniformReal(psi_min, psi_max);

    return v;
}

Eigen::Vector3d RNG::uniformRPY(const Eigen::Vector3d &bounds)
{
    return uniformRPY(-bounds, bounds);
}

Eigen::Vector3d RNG::unifromRPY()
{
    return uniformRPY(Eigen::Vector3d{-constants::pi, -constants::half_pi, -constants::pi},
                      Eigen::Vector3d{constants::pi, constants::half_pi, constants::pi});
}

Eigen::Vector3d RNG::uniformVec(const Eigen::Vector3d &lbound, const Eigen::Vector3d &ubound)
{
    Eigen::Vector3d vec;
    vec[0] = uniformReal(lbound[0], ubound[0]);
    vec[1] = uniformReal(lbound[1], ubound[1]);
    vec[2] = uniformReal(lbound[2], ubound[2]);

    return vec;
}

Eigen::Vector3d RNG::uniformVec(const Eigen::Vector3d &bounds)
{
    return uniformVec(-bounds, bounds);
}

Eigen::Vector3d RNG::gaussianVec(const Eigen::Vector3d &mean, const Eigen::Vector3d &stddev)
{
    Eigen::Vector3d vec;
    vec[0] = gaussian(mean[0], stddev[0]);
    vec[1] = gaussian(mean[1], stddev[1]);
    vec[2] = gaussian(mean[2], stddev[2]);
    return vec;
}

Eigen::Vector3d RNG::gaussianVec(const Eigen::Vector3d &stddev)
{
    Eigen::Vector3d vec;
    vec[0] = gaussian(stddev[0]);
    vec[1] = gaussian(stddev[1]);
    vec[2] = gaussian(stddev[2]);

    return vec;
}
