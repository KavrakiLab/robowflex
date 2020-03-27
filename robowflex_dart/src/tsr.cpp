/* Author: Zachary Kingston */

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>

#include <robowflex_dart/structure.h>
#include <robowflex_dart/tsr.h>

using namespace robowflex::darts;

TSR::TSR(const std::string &target, const std::string &base, const RobotPose &pose,  //
         const Bounds &position, const Bounds &orientation)
  : target_(target)
  , base_(base)
  , pose_(pose)
  , plower_(position.first)
  , pupper_(position.second)
  , olower_(orientation.first)
  , oupper_(orientation.second)
{
}

TSR::TSR(const std::string &target, const RobotPose &pose,  //
         const Bounds &position, const Bounds &orientation)
  : TSR(target, magic::ROOT_FRAME, pose, position, orientation)
{
}

// tight bounds
TSR::TSR(const std::string &target, const std::string &base, const RobotPose &pose)
  : TSR(target, base, pose,
        {Eigen::Vector3d::Constant(-magic::DEFAULT_IK_TOLERANCE),
         Eigen::Vector3d::Constant(magic::DEFAULT_IK_TOLERANCE)},
        {Eigen::Vector3d::Constant(-magic::DEFAULT_IK_TOLERANCE),
         Eigen::Vector3d::Constant(magic::DEFAULT_IK_TOLERANCE)})
{
}

TSR::TSR(const std::string &target, const RobotPose &pose) : TSR(target, magic::ROOT_FRAME, pose)
{
}

TSR::TSR(const std::string &target, const std::string &base) : TSR(target, base, RobotPose::Identity())
{
}

void TSR::setPose(const RobotPose &pose)
{
    pose_ = pose;
}

void TSR::setPosition(const Eigen::Ref<const Eigen::Vector3d> &position)
{
    pose_.translation() = position;
}

void TSR::setRotation(const Eigen::Quaterniond &orientation)
{
    pose_.linear() = orientation.toRotationMatrix();
}

bool TSR::setIKTarget(StructurePtr structure)
{
    auto skeleton = structure->getSkeleton();
    auto tnode = skeleton->getBodyNode(target_);

    if (not tnode)
    {
        std::cout << target_ << " does not exist" << std::endl;
        return false;
    }

    auto ik = tnode->getIK(true);
    dart::dynamics::SimpleFramePtr frame = ik->getTarget();

    if (base_ != magic::ROOT_FRAME)
    {
        auto bnode = skeleton->getBodyNode(base_);
        if (not bnode)
        {
            std::cout << base_ << " does not exist" << std::endl;
            return false;
        }

        frame = frame->clone(bnode);
    }

    frame->setTransform(pose_);
    ik->setTarget(frame);

    auto tsr = ik->setErrorMethod<dart::dynamics::InverseKinematics::TaskSpaceRegion>();

    tsr.setLinearBounds(plower_, pupper_);
    tsr.setAngularBounds(olower_, oupper_);

    return true;
}

bool TSR::solve(StructurePtr structure)
{
    setIKTarget(structure);

    auto skeleton = structure->getSkeleton();
    auto tnode = skeleton->getBodyNode(target_);

    if (not tnode)
    {
        std::cout << target_ << " does not exist" << std::endl;
        return false;
    }

    auto ik = tnode->getIK();

    return ik->solveAndApply();
}
