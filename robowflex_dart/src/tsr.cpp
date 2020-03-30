/* Author: Zachary Kingston */

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>

#include <robowflex_dart/structure.h>
#include <robowflex_dart/robot.h>
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

    grad_ = &ik_->getGradientMethod();

    computeDimension();

    return true;
}

bool TSR::solve()
{
    return ik_->solveAndApply();
}

void TSR::useGroup(const std::string &name)
{
    auto robot = std::dynamic_pointer_cast<Robot>(structure_);
    if (not robot)
        return;

    std::vector<dart::dynamics::DegreeOfFreedom *> dofs;
    auto joints = robot->getGroupJoints(name);
    for (const auto joint : joints)
    {
        std::size_t ndof = joint->getNumDofs();
        for (std::size_t i = 0; i < ndof; ++i)
            dofs.emplace_back(joint->getDof(i));
    }

    ik_->setDofs(dofs);
}

std::size_t TSR::getDimension() const
{
    return dimension_;
}

void TSR::getError(Eigen::Ref<Eigen::VectorXd> vector) const
{
    vector = Eigen::VectorXd::Constant(dimension_, 0);
    auto error = tsr_->computeError();

    std::size_t j = 0;
    for (std::size_t i = 0; i < dimension_; ++i)
    {
        if (indices_[i])
            vector[j++] = error[i];
    }
}

void TSR::getGradient(Eigen::VectorXd &q) const
{
    auto error = tsr_->computeError();
    grad_->computeGradient(error, q);
}

void TSR::getJacobian(Eigen::Ref<Eigen::MatrixXd> jac) const
{
    auto tjac = ik_->computeJacobian();
    jac = Eigen::MatrixXd::Constant(dimension_, tjac.cols(), 0);

    std::size_t j = 0;
    for (std::size_t i = 0; i < dimension_; ++i)
    {
        if (indices_[i])
            jac.row(j++) = tjac.row(i);
    }
}

void TSR::computeDimension()
{
    dimension_ = 0;
    indices_ = std::vector<bool>(6, false);

    for (std::size_t i = 0; i < 3; ++i)
    {
        if (std::abs(oupper_[i] - olower_[i]) < dart::math::constants<double>::two_pi())
        {
            indices_[i] = true;
            dimension_++;
        }

        if (std::isfinite(plower_[i]) or std::isfinite(pupper_[i]))
        {
            indices_[i + 3] = true;
            dimension_++;
        }
    }
}

std::size_t TSR::getNumDofs() const
{
    return ik_->getPositions().size();
}
