/* Author: Zachary Kingston */

#include <robowflex_dart/world.h>
#include <robowflex_dart/joints.h>
#include <robowflex_dart/space.h>

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
    auto joint = getJoint(space_->getWorld());
    for (unsigned int i = 0; i < numDof_; ++i)
    {
        unsigned int j = startIndex_ + i;
        if (j >= joint->getNumDofs())
        {
            // SE3EZ_ERROR("Invalid joint specification. Start %1%, DoF %2%, but max %3%", startIndex_,
            // numDof_,
            //             joint->getNumDofs());
            throw std::runtime_error("Invalid joint");
        }

        auto dof = joint->getDof(i);
        indices_.emplace_back(dof->getIndexInSkeleton());
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

std::size_t Joint::getSkeletonIndex() const
{
    return skelIndex_;
}

void Joint::setJointState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &a) const
{
    auto joint = getJoint(world);
    joint->getSkeleton()->setPositions(indices_, a);
}

void Joint::getJointState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> a) const
{
    auto joint = getJoint(world);
    a = joint->getSkeleton()->getPositions(indices_);
}
