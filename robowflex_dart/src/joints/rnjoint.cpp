/* Author: Zachary Kingston */

#include <robowflex_dart/joints.h>
#include <robowflex_dart/space.h>

using namespace robowflex::darts;

///
/// RnJoint
///

RnJoint::RnJoint(StateSpace *space,             //
                 dart::dynamics::Joint *joint,  //
                 double low, double high)
  : RnJoint(space, joint,                       //
            1, 0,                               //
            Eigen::VectorXd::Constant(1, low),  //
            Eigen::VectorXd::Constant(1, high))
{
}

RnJoint::RnJoint(StateSpace *space,                   //
                 dart::dynamics::Joint *joint,        //
                 unsigned int n, unsigned int start,  //
                 Eigen::VectorXd low, Eigen::VectorXd high)
  : Joint(space, joint, n, start, n), low_(low), high_(high)
{
    if (low_.size() != numDof_ or high_.size() != numDof_)
    {
    }

    for (unsigned int i = 0; i < numDof_; ++i)
        space_->addDimension(low[i], high[i]);
}

// L1
double RnJoint::distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                         const Eigen::Ref<const Eigen::VectorXd> &b) const
{
    return (a - b).lpNorm<1>();
}

double RnJoint::getMaximumExtent() const
{
    return distance(low_, high_);
}

void RnJoint::interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                          const Eigen::Ref<const Eigen::VectorXd> &b,  //
                          double t,                                    //
                          Eigen::Ref<Eigen::VectorXd> c) const
{
    c = a + t * (b - a);
}

void RnJoint::enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const
{
    for (unsigned int i = 0; i < numDof_; ++i)
    {
        double &v = a[i];
        v = (v < low_[i]) ? low_[i] : ((v > high_[i]) ? high_[i] : v);
    }
}

bool RnJoint::satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    for (unsigned int i = 0; i < numDof_; ++i)
    {
        const double &v = a[i];
        if ((v < low_[i]) or (v > high_[i]))
            return false;
    }

    return true;
}

void RnJoint::sample(Eigen::Ref<Eigen::VectorXd> a) const
{
    for (unsigned int i = 0; i < numDof_; ++i)
        a[i] = rng_.uniformReal(low_[i], high_[i]);
}

void RnJoint::sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                         const Eigen::Ref<const Eigen::VectorXd> &near,  //
                         double r) const
{
    for (unsigned int i = 0; i < numDof_; ++i)
        a[i] = rng_.uniformReal(near[i] - r, near[i] + r);

    enforceBounds(a);
}
