/* Author: Zachary Kingston */

#include <robowflex_library/constants.h>

#include <robowflex_dart/joints.h>
#include <robowflex_dart/space.h>

namespace constants = robowflex::constants;
using namespace robowflex::darts;

SO3Joint::SO3Joint(StateSpace *space,  //
                   dart::dynamics::Joint *joint)
  : Joint(space, joint, 4)
{
    space_->addDimension(-1., 1.);  // q w
    space_->addDimension(-1., 1.);  // q x
    space_->addDimension(-1., 1.);  // q y
    space_->addDimension(-1., 1.);  // q z
}

Eigen::Quaterniond SO3Joint::toQuat(const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    return Eigen::Quaterniond(a[0], a[1], a[2], a[3]);
}

void SO3Joint::setQuat(Eigen::Ref<Eigen::VectorXd> a, const Eigen::Quaterniond &q) const
{
    a[0] = q.w();
    a[1] = q.x();
    a[2] = q.y();
    a[3] = q.z();
}

// L1
double SO3Joint::distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                          const Eigen::Ref<const Eigen::VectorXd> &b) const
{
    return toQuat(a).angularDistance(toQuat(b));
}

double SO3Joint::getMaximumExtent() const
{
    return constants::half_pi;
}

void SO3Joint::interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                           const Eigen::Ref<const Eigen::VectorXd> &b,  //
                           double t,                                    //
                           Eigen::Ref<Eigen::VectorXd> c) const
{
    auto slerp = toQuat(a).slerp(t, toQuat(b));
    setQuat(c, slerp);
}

void SO3Joint::enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const
{
    auto q = toQuat(a).normalized();
    setQuat(a, q);
}

bool SO3Joint::satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    double n = a.norm();
    return (n - 1.) < constants::eps;
}

void SO3Joint::sample(Eigen::Ref<Eigen::VectorXd> a) const
{
    double q[4];
    rng_.quaternion(q);
    a[0] = q[3];
    a[1] = q[0];
    a[2] = q[1];
    a[3] = q[2];
}

void SO3Joint::sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                          const Eigen::Ref<const Eigen::VectorXd> &near,  //
                          double r) const
{
    if (r >= constants::quarter_pi)
        sample(a);

    else
    {
        double d = rng_.uniform01();
        auto qnear = toQuat(near);
        Eigen::Quaterniond q(                                                                            //
            Eigen::AngleAxisd(2. * std::pow(d, constants::third) * r,                                    //
                              Eigen::Vector3d{rng_.gaussian01(), rng_.gaussian01(), rng_.gaussian01()})  //
        );
        auto qs = qnear * q;
        setQuat(a, qs);
    }
}

void SO3Joint::setJointState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    auto *joint = getJoint(world);
    auto *j = static_cast<dart::dynamics::FreeJoint *>(joint);

    Eigen::Isometry3d tf;
    tf.translation() = j->getPositions();
    tf.linear() = toQuat(a).toRotationMatrix();

    j->setRelativeTransform(tf);
}

void SO3Joint::getJointState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> a) const
{
    auto *joint = getJoint(world);

    Eigen::Isometry3d tf = joint->getRelativeTransform();
    Eigen::Quaterniond q(tf.linear());

    setQuat(a, q);
}
