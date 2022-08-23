/* Author: Zachary Kingston */

#include <robowflex_dart/joints.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/world.h>

#include <utility>

using namespace robowflex::darts;

///
/// Joint
///

Joint::Joint(StateSpace *space,         //
             unsigned int skelIndex,    //
             unsigned int jointIndex,   //
             unsigned int sizeInSpace,  //
             unsigned int startIndex,   //
             unsigned int numDof)
  : space_(space)
  , rng_(space->rng_)
  , skelIndex_(skelIndex)
  , jointIndex_(jointIndex)
  , startInSpace_(space_->getDimension())
  , sizeInSpace_(sizeInSpace)
  , startIndex_(startIndex)
  , numDof_(numDof)
{
    auto *joint = getJoint(space_->getWorld());
    for (unsigned int i = 0; i < numDof_; ++i)
    {
        unsigned int j = startIndex_ + i;
        if (j >= joint->getNumDofs())
            throw std::runtime_error("Invalid joint");

        auto *dof = joint->getDof(j);
        indices_.emplace_back(dof->getIndexInSkeleton());
        dofs_.emplace_back(dof->getName());
    }
}

Joint::Joint(StateSpace *space, dart::dynamics::Joint *joint, unsigned int sizeInSpace,
             unsigned int startIndex, unsigned int numDof)
  : Joint(space,                                                      //
          space->getWorld()->getSkeletonIndex(joint->getSkeleton()),  //
          joint->getJointIndexInSkeleton(),                           //
          sizeInSpace,                                                //
          startIndex,                                                 //
          (numDof) ? numDof : joint->getNumDofs())
{
}

dart::dynamics::Joint *Joint::getJoint(WorldPtr world) const
{
    const auto &sim = world->getSimConst();
    return sim->getSkeleton(skelIndex_)->getJoint(jointIndex_);
}

Eigen::Ref<Eigen::VectorXd> Joint::getSpaceVars(Eigen::Ref<Eigen::VectorXd> a)
{
    return a.segment(startInSpace_, sizeInSpace_);
}

Eigen::Ref<const Eigen::VectorXd> Joint::getSpaceVarsConst(const Eigen::Ref<const Eigen::VectorXd> &a)
{
    return a.segment(startInSpace_, sizeInSpace_);
}

const std::vector<std::size_t> &Joint::getIndices() const
{
    return indices_;
}

const std::vector<std::string> &Joint::getDofs() const
{
    return dofs_;
}

std::size_t Joint::getSkeletonIndex() const
{
    return skelIndex_;
}

std::size_t Joint::getJointIndex() const
{
    return jointIndex_;
}

std::size_t Joint::getDimension() const
{
    return numDof_;
}

void Joint::setJointState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    auto *joint = getJoint(std::move(world));
    joint->getSkeleton()->setPositions(indices_, a);
}

void Joint::getJointState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> a) const
{
    auto *joint = getJoint(std::move(world));
    a = joint->getSkeleton()->getPositions(indices_);
}

void Joint::setUpperLimits(const Eigen::Ref<const Eigen::VectorXd> & /*v*/)
{
    throw std::runtime_error("Cannot set bounds on this joint!");
}

void Joint::setLowerLimits(const Eigen::Ref<const Eigen::VectorXd> & /*v*/)
{
    throw std::runtime_error("Cannot set bounds on this joint!");
}

Eigen::VectorXd Joint::getUpperLimits() const
{
    return space_->getUpperBound().segment(startInSpace_, sizeInSpace_);
}

Eigen::VectorXd Joint::getLowerLimits() const
{
    return space_->getLowerBound().segment(startInSpace_, sizeInSpace_);
}
