/* Author: Zachary Kingston */

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>

#include <robowflex_library/tf.h>

#include <robowflex_dart/structure.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>

using namespace robowflex::darts;

///
/// TSR
///

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

// mirrored bounds

TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base,  //
         const RobotPose &pose, const Eigen::Vector3d &position, const Eigen::Vector3d &orientation)
  : TSR(structure, target, base, pose, Bounds{-position, position}, Bounds{-orientation, orientation})
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target,  //
         const RobotPose &pose, const Eigen::Vector3d &position, const Eigen::Vector3d &orientation)
  : TSR(structure, target, magic::ROOT_FRAME, pose, position, orientation)
{
}

// tight bounds
TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
         const RobotPose &pose)
  : TSR(structure, target, base, pose, Eigen::Vector3d::Constant(magic::DEFAULT_IK_TOLERANCE),
        Eigen::Vector3d::Constant(magic::DEFAULT_IK_TOLERANCE))
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const RobotPose &pose)
  : TSR(structure, target, magic::ROOT_FRAME, pose)
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
         const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation)
  : TSR(structure, target, base, robowflex::TF::createPoseQ(position, rotation))
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const Eigen::Vector3d &position,
         const Eigen::Quaterniond &rotation)
  : TSR(structure, target, magic::ROOT_FRAME, position, rotation)
{
}

TSR::TSR(const StructurePtr &structure, const std::string &target, const std::string &base)
  : TSR(structure, target, base, RobotPose::Identity())
{
}

TSR::~TSR()
{
    if (node_)
        node_->clearIK();
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
    frame_->setRelativeTransform(pose_);
}

bool TSR::initialize()
{
    auto skeleton = structure_->getSkeleton();
    node_ = skeleton->getBodyNode(target_);

    if (not node_)
    {
        std::cout << target_ << " does not exist" << std::endl;
        return false;
    }

    ik_ = node_->getIK(true);
    frame_ = ik_->getTarget();

    bool heir = false;
    if (base_ != magic::ROOT_FRAME)
    {
        auto bnode = skeleton->getBodyNode(base_);
        if (not bnode)
        {
            std::cout << base_ << " does not exist" << std::endl;
            return false;
        }

        heir = true;
        frame_ = frame_->clone(bnode);
    }

    ik_->setTarget(frame_);

    updateTarget();

    tsr_ = &ik_->setErrorMethod<dart::dynamics::InverseKinematics::TaskSpaceRegion>();

    tsr_->setComputeFromCenter(false);

    tsr_->setLinearBounds(plower_, pupper_);
    tsr_->setAngularBounds(olower_, oupper_);

    grad_ = &ik_->getGradientMethod();

    computeDimension();

    if (heir)
        ik_->setHierarchyLevel(1);

    ik_->getSolver()->setTolerance(1e-9);
    ik_->getSolver()->setNumMaxIterations(500);

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

    useIndices(robot->getGroupIndices(name));
}

void TSR::useIndices(const std::vector<std::size_t> &indices)
{
    ik_->setDofs(indices);
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

double TSR::distance() const
{
    Eigen::VectorXd f(dimension_);
    getError(f);
    return f.norm();
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

const StructurePtr &TSR::getStructure() const
{
    return structure_;
}

void TSR::setStructure(const StructurePtr &structure)
{
    structure_ = structure;
    initialize();
}

///
/// TSRConstraint
///

TSRConstraint::TSRConstraint(const StateSpacePtr &space, const TSRPtr &tsr)
  : ompl::base::Constraint(space->getDimension(), tsr->getDimension(), 1e-4), space_(space), tsr_(tsr)
{
    tsr_->useIndices(space_->getIndices());
}

void TSRConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::VectorXd> out) const
{
    space_->setWorldState(space_->getWorld(), x);
    tsr_->getError(out);
}

void TSRConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::MatrixXd> out) const
{
    space_->setWorldState(space_->getWorld(), x);
    tsr_->getJacobian(out);
}

bool TSRConstraint::project(Eigen::Ref<Eigen::VectorXd> x) const
{
    // return ompl::base::Constraint::project(x);
    space_->setWorldState(space_->getWorld(), x);
    bool r = tsr_->solve();
    space_->getWorldState(space_->getWorld(), x);
    return r;
}

///
/// TSRCompositeConstraint
///

TSRCompositeConstraint::TSRCompositeConstraint(const StateSpacePtr &space, const std::vector<TSRPtr> &tsrs)
  : ompl::base::Constraint(space->getDimension(),
                           [&tsrs] {
                               std::size_t k = 0;
                               for (const auto &tsr : tsrs)
                                   k += tsr->getDimension();
                               return k;
                           }(),
                           0.05)
  , space_(space)
  , tsrs_(tsrs)
{
    setMaxIterations(1000);

    for (const auto &tsr : tsrs_)
        structures_.emplace(tsr->getStructure());
}

void TSRCompositeConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                      Eigen::Ref<Eigen::VectorXd> out) const
{
    space_->setWorldState(space_->getWorld(), x);
    unsigned int i = 0;
    for (auto tsr : tsrs_)
    {
        tsr->getError(out.segment(i, tsr->getDimension()));
        i += tsr->getDimension();
    }
}

void TSRCompositeConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                                      Eigen::Ref<Eigen::MatrixXd> out) const
{
    space_->setWorldState(space_->getWorld(), x);
    unsigned int i = 0;
    for (auto tsr : tsrs_)
    {
        tsr->getJacobian(out.block(i, 0, tsr->getDimension(), n_));
        i += tsr->getDimension();
    }
}

bool TSRCompositeConstraint::project(Eigen::Ref<Eigen::VectorXd> x) const
{
    // Newton's method
    unsigned int iter = 0;
    double norm = 0;
    Eigen::VectorXd f(getCoDimension());
    Eigen::MatrixXd j(getCoDimension(), n_);

    const double squaredTolerance = tolerance_ * tolerance_;

    function(x, f);
    while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < maxIterations_)
    {
        jacobian(x, j);
        x -= 0.5 * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        function(x, f);
    }

    return norm < squaredTolerance;

    // return ompl::base::Constraint::project(x);

    // space_->setWorldState(space_->getWorld(), x);

    // bool r = true;
    // for (auto &structure : structures_)
    //     r &= structure->solveIK();

    // space_->getWorldState(space_->getWorld(), x);
    // return r;
}
