/* Author: Zachary Kingston */

#include <robowflex_dart/constants.h>
#include <robowflex_dart/joints.h>
#include <robowflex_dart/space.h>

using namespace robowflex::darts;

///
/// SO2Joint
///

SO2Joint::SO2Joint(StateSpace *space,  //
                   dart::dynamics::Joint *joint)
  : Joint(space, joint, 1)
{
    space_->addDimension(-constants::pi, constants::pi);
}

// L1
double SO2Joint::distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                          const Eigen::Ref<const Eigen::VectorXd> &b) const
{
    double d = std::fabs(a[0] - b[0]);
    return (d > constants::pi) ? 2.0 * constants::pi - d : d;
}

double SO2Joint::getMaximumExtent() const
{
    return constants::pi;
}

void SO2Joint::interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                           const Eigen::Ref<const Eigen::VectorXd> &b,  //
                           double t,                                    //
                           Eigen::Ref<Eigen::VectorXd> c) const
{
    double diff = b[0] - a[0];
    if (std::fabs(diff) <= constants::pi)
        c[0] = a[0] + diff * t;
    else
    {
        if (diff > 0.0)
            diff = 2.0 * constants::pi - diff;
        else
            diff = -2.0 * constants::pi - diff;

        double &v = c[0];
        v = a[0] - diff * t;

        if (v > constants::pi)
            v -= 2.0 * constants::pi;
        else if (v < -constants::pi)
            v += 2.0 * constants::pi;
    }
}

void SO2Joint::enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const
{
    double &v = a[0];
    v = std::fmod(v, 2.0 * constants::pi);
    if (v < -constants::pi)
        v += 2.0 * constants::pi;
    else if (v >= constants::pi)
        v -= 2.0 * constants::pi;
}

bool SO2Joint::satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    const double &v = a[0];
    return (v >= -constants::pi) and (v <= constants::pi);
}

void SO2Joint::sample(Eigen::Ref<Eigen::VectorXd> a) const
{
    a[0] = rng_.uniformReal(-constants::pi, constants::pi);
}

void SO2Joint::sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                          const Eigen::Ref<const Eigen::VectorXd> &near,  //
                          double r) const
{
    a[0] = rng_.uniformReal(near[0] - r, near[0] + r);
    enforceBounds(a);
}
