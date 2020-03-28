/* Author: Zachary Kingston */

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>

#include <robowflex_dart/structure.h>
#include <robowflex_dart/tsr.h>

using namespace robowflex::darts;

TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
         const RobotPose &pose, const Bounds &position, const Bounds &orientation)
  : structure_(structure)
  , target_(target)
  , base_(base)
  , pose_(pose)
  , plower_(position.first)
  , pupper_(position.second)
  , olower_(orientation.first)
  , oupper_(orientation.second)
{
    initialize();
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const RobotPose &pose,  //
         const Bounds &position, const Bounds &orientation)
  : TSR(structure, target, magic::ROOT_FRAME, pose, position, orientation)
{
}

// tight bounds
TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
         const RobotPose &pose)
  : TSR(structure, target, base, pose,
        {Eigen::Vector3d::Constant(-magic::DEFAULT_IK_TOLERANCE),
         Eigen::Vector3d::Constant(magic::DEFAULT_IK_TOLERANCE)},
        {Eigen::Vector3d::Constant(-magic::DEFAULT_IK_TOLERANCE),
         Eigen::Vector3d::Constant(magic::DEFAULT_IK_TOLERANCE)})
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const RobotPose &pose)
  : TSR(structure, target, magic::ROOT_FRAME, pose)
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base)
  : TSR(structure, target, base, RobotPose::Identity())
{
}

void TSR::setPose(const RobotPose &pose)
{
    pose_ = pose;
    updateTarget();
}

void TSR::setPosition(const Eigen::Ref<const Eigen::Vector3d> &position)
{
    pose_.translation() = position;
    updateTarget();
}

void TSR::setRotation(const Eigen::Quaterniond &orientation)
{
    pose_.linear() = orientation.toRotationMatrix();
    updateTarget();
}

void TSR::updateTarget()
{
    frame_->setTransform(pose_);
}

bool TSR::initialize()
{
    auto skeleton = structure_->getSkeleton();
    auto tnode = skeleton->getBodyNode(target_);

    if (not tnode)
    {
        std::cout << target_ << " does not exist" << std::endl;
        return false;
    }

    ik_ = tnode->getIK(true);
    frame_ = ik_->getTarget();

    if (base_ != magic::ROOT_FRAME)
    {
        auto bnode = skeleton->getBodyNode(base_);
        if (not bnode)
        {
            std::cout << base_ << " does not exist" << std::endl;
            return false;
        }

        frame_ = frame_->clone(bnode);
    }

    ik_->setTarget(frame_);

    updateTarget();

    tsr_ = &ik_->setErrorMethod<dart::dynamics::InverseKinematics::TaskSpaceRegion>();

    tsr_->setLinearBounds(plower_, pupper_);
    tsr_->setAngularBounds(olower_, oupper_);

    return true;
}

bool TSR::solve()
{
    return ik_->solveAndApply();
}
