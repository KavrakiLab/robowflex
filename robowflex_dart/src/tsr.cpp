/* Author: Zachary Kingston */

#include <boost/format.hpp>

#include <dart/dynamics/InverseKinematics.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <robowflex_library/constants.h>
#include <robowflex_library/log.h>
#include <robowflex_library/tf.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/structure.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

namespace constants = robowflex::constants;
using namespace robowflex::darts;

///
/// TSR::Specification
///

TSR::Specification::Specification(const std::string &structure, const std::string &target_frame,
                                  const Eigen::Ref<const Eigen::Vector3d> &position,
                                  const Eigen::Quaterniond &rotation)
{
    setTarget(structure, target_frame);
    setPose(position, rotation);
}

void TSR::Specification::setTarget(const std::string &structure, const std::string &frame)
{
    target.structure = structure;
    target.frame = frame;

    if (base.structure.empty())
        base.structure = structure;
}

void TSR::Specification::setBase(const std::string &structure, const std::string &frame)
{
    base.structure = structure;
    base.frame = frame;
}

void TSR::Specification::setFrame(const std::string &structure, const std::string &target_frame,
                                  const std::string &base_frame)
{
    target.structure = structure;
    base.structure = structure;

    target.frame = target_frame;
    base.frame = base_frame;
}

void TSR::Specification::addSuffix(const std::string &suffix)
{
    target.structure = target.structure + suffix;
    base.structure = base.structure + suffix;
}

void TSR::Specification::setPosition(const Eigen::Ref<const Eigen::Vector3d> &position)
{
    pose.translation() = position;
}

void TSR::Specification::setPosition(double x, double y, double z)
{
    setPosition(Eigen::Vector3d(x, y, z));
}

void TSR::Specification::setRotation(const Eigen::Quaterniond &orientation)
{
    pose.linear() = orientation.toRotationMatrix();
}

void TSR::Specification::setRotation(double w, double x, double y, double z)
{
    setRotation(Eigen::Quaterniond(w, x, y, z));
}

void TSR::Specification::setRotation(double x, double y, double z)
{
    auto n = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *  //
             Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *  //
             Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());

    setRotation(n);
}

void TSR::Specification::setPose(const RobotPose &other)
{
    pose = other;
}

void TSR::Specification::setPose(const Eigen::Ref<const Eigen::Vector3d> &position,
                                 const Eigen::Quaterniond &rotation)
{
    pose = TF::createPoseQ(position, rotation);
}

void TSR::Specification::setPose(double xp, double yp, double zp, double wr, double xr, double yr, double zr)
{
    setPosition(xp, yp, zp);
    setRotation(wr, xr, yr, zr);
}

void TSR::Specification::setPoseFromWorld(const WorldPtr &world)
{
    const auto &sim = world->getSim();

    const auto &tskl = sim->getSkeleton(target.structure);
    const auto &tbn = tskl->getBodyNode(target.frame);

    if (base.frame != magic::ROOT_FRAME)
    {
        const auto &bskl = sim->getSkeleton(base.structure);
        const auto &bbn = bskl->getBodyNode(base.frame);

        pose = tbn->getTransform(bbn);
    }
    else
        pose = tbn->getTransform();
}

void TSR::Specification::setXPosTolerance(double bound)
{
    setXPosTolerance(-bound, bound);
}

void TSR::Specification::setYPosTolerance(double bound)
{
    setYPosTolerance(-bound, bound);
}

void TSR::Specification::setZPosTolerance(double bound)
{
    setZPosTolerance(-bound, bound);
}

void TSR::Specification::setXPosTolerance(double lower, double upper)
{
    position.lower[0] = lower;
    position.upper[0] = upper;
    fixBounds();

    indices[3] = isPosConstrained(lower, upper);
    dimension = getDimension();
}

void TSR::Specification::setYPosTolerance(double lower, double upper)
{
    position.lower[1] = lower;
    position.upper[1] = upper;
    fixBounds();

    indices[4] = isPosConstrained(lower, upper);
    dimension = getDimension();
}

void TSR::Specification::setZPosTolerance(double lower, double upper)
{
    position.lower[2] = lower;
    position.upper[2] = upper;
    fixBounds();

    indices[5] = isPosConstrained(lower, upper);
    dimension = getDimension();
}

void TSR::Specification::setXRotTolerance(double bound)
{
    setXRotTolerance(-bound, bound);
}

void TSR::Specification::setYRotTolerance(double bound)
{
    setYRotTolerance(-bound, bound);
}

void TSR::Specification::setZRotTolerance(double bound)
{
    setZRotTolerance(-bound, bound);
}

void TSR::Specification::setXRotTolerance(double lower, double upper)
{
    orientation.lower[0] = lower;
    orientation.upper[0] = upper;
    fixBounds();

    indices[0] = isRotConstrained(lower, upper);
    dimension = getDimension();
}

void TSR::Specification::setYRotTolerance(double lower, double upper)
{
    orientation.lower[1] = lower;
    orientation.upper[1] = upper;
    fixBounds();

    indices[1] = isRotConstrained(lower, upper);
    dimension = getDimension();
}

void TSR::Specification::setZRotTolerance(double lower, double upper)
{
    orientation.lower[2] = lower;
    orientation.upper[2] = upper;
    fixBounds();

    indices[2] = isRotConstrained(lower, upper);
    dimension = getDimension();
}

void TSR::Specification::setNoXPosTolerance()
{
    setXPosTolerance(std::numeric_limits<double>::infinity());
}

void TSR::Specification::setNoYPosTolerance()
{
    setYPosTolerance(std::numeric_limits<double>::infinity());
}

void TSR::Specification::setNoZPosTolerance()
{
    setZPosTolerance(std::numeric_limits<double>::infinity());
}

void TSR::Specification::setNoPosTolerance()
{
    setNoXPosTolerance();
    setNoYPosTolerance();
    setNoZPosTolerance();
}

void TSR::Specification::setNoXRotTolerance()
{
    setXRotTolerance(constants::pi);
}

void TSR::Specification::setNoYRotTolerance()
{
    setYRotTolerance(constants::pi);
}

void TSR::Specification::setNoZRotTolerance()
{
    setZRotTolerance(constants::pi);
}

void TSR::Specification::setNoRotTolerance()
{
    setNoXRotTolerance();
    setNoYRotTolerance();
    setNoZRotTolerance();
}

void TSR::Specification::fixBounds()
{
    {
        Eigen::Vector3d u, l;
        for (std::size_t i = 0; i < 3; ++i)
        {
            u[i] = std::max(position.lower[i], position.upper[i]);
            l[i] = std::min(position.lower[i], position.upper[i]);
        }

        position.lower = l;
        position.upper = u;
    }

    {
        Eigen::Vector3d u, l;
        for (std::size_t i = 0; i < 3; ++i)
        {
            u[i] = std::max({orientation.lower[i], orientation.upper[i], -constants::pi});
            l[i] = std::min({orientation.lower[i], orientation.upper[i], constants::pi});
        }

        orientation.lower = l;
        orientation.upper = u;
    }
}

std::size_t TSR::Specification::getDimension() const
{
    std::size_t k = 0;
    for (const auto &idx : indices)
    {
        if (idx)
            k++;
    }

    return k;
}

bool TSR::Specification::isPosConstrained(double lower, double upper) const
{
    return std::isfinite(lower) or std::isfinite(upper);
}

bool TSR::Specification::isRotConstrained(double lower, double upper) const
{
    return std::abs(upper - lower) < constants::two_pi;
}

Eigen::Vector3d TSR::Specification::getPosition() const
{
    return pose.translation();
}

Eigen::Quaterniond TSR::Specification::getRotation() const
{
    return TF::getPoseRotation(pose);
}

Eigen::Vector3d TSR::Specification::getEulerRotation() const
{
    return getRotation().toRotationMatrix().eulerAngles(0, 1, 2);
}

bool TSR::Specification::intersect(const Specification &other)
{
    // must be same reference frame
    if (target.structure != other.target.structure or target.frame != other.target.frame)
        return false;

    if (base.structure != other.base.structure or base.frame != other.base.frame)
        return false;

    // TODO: Check if rotations overlap
    // if (getRotation().angularDistance(other.getRotation()) > magic::DEFAULT_IK_TOLERANCE)
    //     return false;

    Eigen::Vector3d p = getPosition();
    Eigen::Vector3d op = other.getPosition();

    // check if positions overlap
    for (std::size_t i = 0; i < 3; ++i)
    {
        double pi = p[i], piu = position.upper[i], pil = position.lower[i];
        double opi = op[i], opiu = other.position.upper[i], opil = other.position.lower[i];

        //   pi---------> piu
        // --|------|------|------|-- axis
        //        opil <---------opi
        // Check if overlap

        if (pi < opi)
            if ((pi + piu) < (opi + opil))
                return false;

        //  opi--------> opiu
        // --|------|------|------|-- axis
        //         pil <----------pi
        // Check if overlap

        if (pi > opi)
            if ((pi + pil) > (opi + opiu))
                return false;
    }

    Eigen::Vector3d np;

    if (not isRotationConstrained())
        setRotation(other.getRotation());

    // enforce new bounds
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (other.orientation.lower[i] > orientation.lower[i])
            orientation.lower[i] = other.orientation.lower[i];
        if (other.orientation.upper[i] < orientation.upper[i])
            orientation.upper[i] = other.orientation.upper[i];

        double pi = p[i], piu = position.upper[i], pil = position.lower[i];
        double opi = op[i], opiu = other.position.upper[i], opil = other.position.lower[i];

        double low = std::max(pi + pil, opi + opil);
        double high = std::min(pi + piu, opi + opiu);

        np[i] = (high + low) / 2.;
        double v = std::fabs(high - np[i]);

        position.upper[i] = v;
        position.lower[i] = -v;

        indices[i] = isRotConstrained(orientation.lower[i], orientation.upper[i]);
        indices[3 + i] = isPosConstrained(-v, v);
    }

    dimension = getDimension();
    setPosition(np);

    return true;
}

bool TSR::Specification::isRelative() const
{
    return base.frame != magic::ROOT_FRAME;
}

bool TSR::Specification::isPositionConstrained() const
{
    bool value = false;
    for (std::size_t i = 0; i < 3; ++i)
        value |= isPosConstrained(position.lower[i], position.upper[i]);

    return value;
}

bool TSR::Specification::isRotationConstrained() const
{
    bool value = false;
    for (std::size_t i = 0; i < 3; ++i)
        value |= isRotConstrained(orientation.lower[i], orientation.upper[i]);

    return value;
}

void TSR::Specification::print(std::ostream &out) const
{
    if (isRelative())
        out << boost::format("B:%1%:%2% => T:%3%:%4%")  //
                   % base.frame % base.structure % target.frame % target.structure;
    else
        out << boost::format("T:%1%:%2%")  //
                   % target.frame % target.structure;

    out << std::endl;
    out << "Position                          Orientation" << std::endl;

    auto p = getPosition();
    auto o = getRotation();
    out << boost::format(                                                                              //
               " x:%1$+07.4f (%2$+07.4f, %3$+07.4f) [%7%]  x:%4$+07.4f (%5$+07.4f, %6$+07.4f) [%8%]")  //
               % p[0] % position.lower[0] % position.upper[0]                                          //
               % o.x() % orientation.lower[0] % orientation.upper[0]                                   //
               % indices[3] % indices[0];
    out << std::endl;
    out << boost::format(                                                                              //
               " y:%1$+07.4f (%2$+07.4f, %3$+07.4f) [%7%]  y:%4$+07.4f (%5$+07.4f, %6$+07.4f) [%8%]")  //
               % p[1] % position.lower[1] % position.upper[1]                                          //
               % o.y() % orientation.lower[1] % orientation.upper[1]                                   //
               % indices[4] % indices[1];
    out << std::endl;
    out << boost::format(                                                                              //
               " z:%1$+07.4f (%2$+07.4f, %3$+07.4f) [%7%]  z:%4$+07.4f (%5$+07.4f, %6$+07.4f) [%8%]")  //
               % p[2] % position.lower[2] % position.upper[2]                                          //
               % o.z() % orientation.lower[2] % orientation.upper[2]                                   //
               % indices[5] % indices[2];
    out << std::endl;
    out << boost::format("                                   w:%1$+07.4f") % o.w();
    out << std::endl;
}

///
/// TSR
///

TSR::TSR(const WorldPtr &world, const Specification &spec) : world_(world), spec_(spec)
{
}

TSR::~TSR()
{
    clear();
}

void TSR::clear()
{
    if (tnd_ and ik_)
        tnd_->clearIK();

    frame_ = nullptr;
    ik_ = nullptr;
    tnd_ = nullptr;
    tsr_ = nullptr;
}

void TSR::setWorld(const WorldPtr &world)
{
    clear();
    world_ = world;
    initialize();
}

void TSR::useGroup(const std::string &name)
{
    auto robot = world_->getRobot(spec_.target.structure);
    if (not robot)
        throw std::runtime_error("Target robot does exist in world!");

    useIndices(robot->getGroupIndices(name));
}

void TSR::useIndices(const std::vector<std::size_t> &indices)
{
    indices_ = indices;
    if (ik_)
    {
        ik_->setDofs(indices_);
        indices_ = ik_->getDofs();
    }

    computeBijection();
}

void TSR::useWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices)
{
    std::vector<std::size_t> use;
    for (const auto &index : indices)
    {
        if (index.first == getSkeletonIndex())
            use.emplace_back(index.second);
    }

    useIndices(use);
}

void TSR::setWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices)
{
    world_indices_ = indices;
    computeBijection();
}

std::size_t TSR::getSkeletonIndex()
{
    if (not tnd_)
        initialize();

    return skel_index_;
}

const std::vector<std::size_t> &TSR::getIndices() const
{
    return indices_;
}

std::vector<std::pair<std::size_t, std::size_t>> TSR::computeWorldIndices() const
{
    std::vector<std::pair<std::size_t, std::size_t>> wi;
    for (const auto &index : indices_)
        wi.emplace_back(skel_index_, index);

    return wi;
}

const std::vector<std::pair<std::size_t, std::size_t>> &TSR::getWorldIndices() const
{
    return world_indices_;
}

std::size_t TSR::getDimension() const
{
    return spec_.dimension;
}

std::size_t TSR::getNumDofs() const
{
    return indices_.size();
}

std::size_t TSR::getNumWorldDofs() const
{
    return world_indices_.size();
}

robowflex::RobotPose TSR::getTransformToFrame() const
{
    const auto &sim = world_->getSim();
    const auto &bskl = sim->getSkeleton(spec_.base.structure);
    auto *bnd = bskl->getBodyNode(spec_.base.frame);

    if (not tnd_)
        throw std::runtime_error("Target body node is not initialized");

    return tnd_->getTransform(bnd);
}

void TSR::getErrorWorldRaw(Eigen::Ref<Eigen::VectorXd> error) const
{
    world_->lock();
    error = tsr_->computeError();
    world_->unlock();
}

void TSR::getErrorWorld(Eigen::Ref<Eigen::VectorXd> error) const
{
    world_->lock();

    auto tsr_error = tsr_->computeError();

    std::size_t j = 0;
    for (std::size_t i = 0; i < 6; ++i)
    {
        if (spec_.indices[i])
            error[j++] = tsr_error[i];
    }

    world_->unlock();
}

void TSR::getErrorWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                             Eigen::Ref<Eigen::VectorXd> error) const
{
    if (bijection_.empty())
        getError(world, error);
    else
    {
        Eigen::VectorXd state(getNumDofs());
        fromBijection(state, world);
        getError(state, error);
    }
}

void TSR::getError(const Eigen::Ref<const Eigen::VectorXd> &state, Eigen::Ref<Eigen::VectorXd> error) const
{
    setPositions(state);
    getErrorWorld(error);
}

void TSR::getJacobianWorld(Eigen::Ref<Eigen::MatrixXd> jacobian) const
{
    world_->lock();

    auto tjac = ik_->computeJacobian();

    std::size_t j = 0;
    for (std::size_t i = 0; i < 6; ++i)
    {
        if (spec_.indices[i])
            jacobian.row(j++) = tjac.row(i);
    }

    world_->unlock();
}

void TSR::getJacobianWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                Eigen::Ref<Eigen::MatrixXd> jacobian) const
{
    if (bijection_.empty())
        getJacobian(world, jacobian);
    else
    {
        Eigen::VectorXd state(getNumDofs());
        fromBijection(state, world);

        Eigen::VectorXd tjac(getDimension(), getNumDofs());
        getJacobian(state, tjac);

        for (std::size_t i = 0; i < bijection_.size(); ++i)
            if (bijection_[i] < world_indices_.size())
                jacobian.col(bijection_[i]) = tjac.col(i);
    }
}

void TSR::getJacobian(const Eigen::Ref<const Eigen::VectorXd> &state,
                      Eigen::Ref<Eigen::MatrixXd> jacobian) const
{
    world_->lock();
    setPositions(state);
    getJacobianWorld(jacobian);
    world_->unlock();
}

double TSR::distanceWorld() const
{
    Eigen::VectorXd x(getDimension());
    getErrorWorld(x);
    return x.norm();
}

double TSR::distanceWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const
{
    Eigen::VectorXd x(getDimension());
    getErrorWorldState(world, x);
    return x.norm();
}

double TSR::distance(const Eigen::Ref<const Eigen::VectorXd> &state) const
{
    Eigen::VectorXd x(getDimension());
    getError(state, x);
    return x.norm();
}

bool TSR::solveWorld()
{
    if (not ik_)
    {
        RBX_ERROR("TSR: Solve called before initialize!");
        return false;
    }

    world_->lock();
    bool r = ik_->solveAndApply();
    world_->unlock();
    return r;
}

bool TSR::solveWorldState(Eigen::Ref<Eigen::VectorXd> world)
{
    if (bijection_.empty())
        return solve(world);

    Eigen::VectorXd state(getNumDofs());
    fromBijection(state, world);
    bool r = solve(state);
    toBijection(world, state);
    return r;
}

bool TSR::solve(Eigen::Ref<Eigen::VectorXd> state)
{
    world_->lock();

    setPositions(state);
    bool r = solveWorld();
    getPositions(state);

    world_->unlock();
    return r;
}

bool TSR::solveGradientWorld()
{
    world_->lock();

    Eigen::VectorXd state = ik_->getPositions();
    bool r = solveGradient(state);
    setPositions(state);

    world_->unlock();
    return r;
}

bool TSR::solveGradientWorldState(Eigen::Ref<Eigen::VectorXd> world)
{
    if (bijection_.empty())
        return solveGradient(world);

    Eigen::VectorXd state(getNumDofs());
    fromBijection(state, world);
    bool r = solveGradient(state);
    toBijection(world, state);
    return r;
}

bool TSR::solveGradient(Eigen::Ref<Eigen::VectorXd> state)
{
    world_->lock();

    unsigned int iter = 0;
    double norm = 0;
    Eigen::VectorXd f(getDimension());
    Eigen::MatrixXd j(getDimension(), getNumDofs());

    const double squared_tolerance = spec_.tolerance * spec_.tolerance;

    setPositions(state);
    getErrorWorld(f);

    while ((norm = f.norm()) > squared_tolerance && iter++ < spec_.maxIter)
    {
        getJacobianWorld(j);
        state -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);

        setPositions(state);
        getErrorWorld(f);
    }

    world_->unlock();

    return norm < squared_tolerance;
}

void TSR::setPositionsWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const
{
    Eigen::VectorXd state(getNumDofs());
    fromBijection(state, world);
    setPositions(state);
}

void TSR::setPositions(const Eigen::Ref<const Eigen::VectorXd> &state) const
{
    world_->lock();
    ik_->setPositions(state);
    world_->unlock();
}

void TSR::getPositionsWorldState(Eigen::Ref<Eigen::VectorXd> world) const
{
    Eigen::VectorXd state(getNumDofs());
    getPositions(state);
    toBijection(world, state);
}

void TSR::getPositions(Eigen::Ref<Eigen::VectorXd> state) const
{
    world_->lock();
    state = ik_->getPositions();
    world_->unlock();
}

TSR::Specification &TSR::getSpecification()
{
    return spec_;
}

void TSR::updatePose()
{
    if (frame_)
        frame_->setRelativeTransform(spec_.pose);
}

void TSR::updateBounds()
{
    if (tsr_)
    {
        tsr_->setLinearBounds(spec_.position.lower, spec_.position.upper);
        tsr_->setAngularBounds(spec_.orientation.lower, spec_.orientation.upper);
    }
}

void TSR::updateSolver()
{
    if (ik_)
    {
        ik_->getSolver()->setTolerance(spec_.tolerance);
        ik_->getSolver()->setNumMaxIterations(spec_.maxIter);
    }
}

void TSR::initialize()
{
    const auto &sim = world_->getSim();
    const auto &tskl = sim->getSkeleton(spec_.target.structure);
    if (not tskl)
        throw std::runtime_error("Target skeleton " + spec_.target.structure + " in TSR does not exist!");
    skel_index_ = world_->getSkeletonIndex(tskl);

    const auto &bskl = sim->getSkeleton(spec_.base.structure);
    if (not bskl)
        throw std::runtime_error("Base skeleton " + spec_.base.structure + " in TSR does not exist!");

    tnd_ = tskl->getBodyNode(spec_.target.frame);
    ik_ = tnd_->getIK(true);
    frame_ = ik_->getTarget();

    if (spec_.base.frame != magic::ROOT_FRAME)
    {
        auto *bnd = bskl->getBodyNode(spec_.base.frame);
        frame_ = frame_->clone(bnd);
    }

    ik_->setTarget(frame_);

    tsr_ = &ik_->setErrorMethod<dart::dynamics::InverseKinematics::TaskSpaceRegion>();
    tsr_->setComputeFromCenter(false);

    updatePose();
    updateBounds();
    updateSolver();

    if (indices_.empty())
        indices_ = ik_->getDofs();
    else
        ik_->setDofs(indices_);
}

void TSR::toBijection(Eigen::Ref<Eigen::VectorXd> world, const Eigen::Ref<const Eigen::VectorXd> &state) const
{
    if (bijection_.empty())
        world = state;
    else
        for (std::size_t i = 0; i < bijection_.size(); ++i)
            if (bijection_[i] < world_indices_.size())
                world[bijection_[i]] = state[i];
}

void TSR::fromBijection(Eigen::Ref<Eigen::VectorXd> state,
                        const Eigen::Ref<const Eigen::VectorXd> &world) const
{
    if (bijection_.empty())
        state = world;
    else
        for (std::size_t i = 0; i < bijection_.size(); ++i)
            if (bijection_[i] < world_indices_.size())
                state[i] = world[bijection_[i]];
}

void TSR::computeBijection()
{
    if (world_indices_.empty())
        return;

    bijection_.resize(indices_.size());

    bool same = world_indices_.size() == indices_.size();
    for (std::size_t i = 0; i < indices_.size(); ++i)
    {
        bijection_[i] = world_indices_.size();
        for (std::size_t j = 0; j < world_indices_.size(); ++j)
        {
            auto entry = world_indices_[j];
            if (entry.first == getSkeletonIndex() and entry.second == indices_[i])
            {
                bijection_[i] = j;
                same &= i == j;
                break;
            }
        }
    }

    if (same)
        bijection_.clear();
}

///
/// TSRSet
///

TSRSet::TSRSet(const WorldPtr &world, const TSRPtr &tsr) : world_(world)
{
    addTSR(tsr);
}

TSRSet::TSRSet(const WorldPtr &world, const std::vector<TSRPtr> &tsrs, bool intersect) : world_(world)
{
    for (const auto &tsr : tsrs)
        addTSR(tsr, intersect);
}

void TSRSet::addTSR(const TSRPtr &tsr, bool intersect, double weight)
{
    TSRPtr ntsr = tsr;
    TSR::Specification spec = tsr->getSpecification();
    if (intersect)
    {
        for (auto &etsr : tsrs_)
            // Don't need this entire TSR if we can intersect
            if (etsr->getSpecification().intersect(spec))
            {
                dimension_ = 0;
                for (auto &tsr : tsrs_)
                    dimension_ += tsr->getDimension();
                return;
            }

        // copy for intersections later
        ntsr = std::make_shared<TSR>(world_, spec);
        ntsr->useIndices(tsr->getIndices());
        ntsr->setWorldIndices(tsr->getWorldIndices());

        // weight relative frames less
        if ((weight - 1.) < constants::eps)
            if (spec.isRelative())
                weight = 0.1;
    }

    tsrs_.emplace_back(ntsr);
    weights_.emplace_back(weight);

    dimension_ += ntsr->getDimension();
    tolerance_ = std::min(tolerance_, spec.tolerance);
}

std::size_t TSRSet::numTSRs() const
{
    return tsrs_.size();
}

const std::vector<TSRPtr> &TSRSet::getTSRs() const
{
    return tsrs_;
}

void TSRSet::setWorld(const WorldPtr &world)
{
    for (auto &tsr : tsrs_)
        tsr->setWorld(world);

    world_ = world;
    initialize();
}

void TSRSet::addSuffix(const std::string &suffix)
{
    for (auto &tsr : tsrs_)
        tsr->getSpecification().addSuffix(suffix);
}

void TSRSet::useGroup(const std::string &name)
{
    for (auto &tsr : tsrs_)
        tsr->useGroup(name);
}

void TSRSet::useIndices(const std::vector<std::size_t> &indices)
{
    for (auto &tsr : tsrs_)
        tsr->useIndices(indices);
}

void TSRSet::useWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices)
{
    for (auto &tsr : tsrs_)
        tsr->useWorldIndices(indices);
}

void TSRSet::setWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices)
{
    for (auto &tsr : tsrs_)
        tsr->setWorldIndices(indices);
}

std::size_t TSRSet::getDimension() const
{
    return dimension_;
}

void TSRSet::getErrorWorld(Eigen::Ref<Eigen::VectorXd> error) const
{
    world_->lock();

    std::size_t i = 0;
    for (std::size_t j = 0; j < tsrs_.size(); ++j)
    {
        const auto &tsr = tsrs_[j];
        tsr->getErrorWorld(error.segment(i, tsr->getDimension()));
        error.segment(i, tsr->getDimension()) *= weights_[j];

        i += tsr->getDimension();
    }

    world_->unlock();
}

void TSRSet::getErrorWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                Eigen::Ref<Eigen::VectorXd> error) const
{
    world_->lock();

    std::size_t i = 0;
    for (std::size_t j = 0; j < tsrs_.size(); ++j)
    {
        const auto &tsr = tsrs_[j];
        tsr->getErrorWorldState(world, error.segment(i, tsr->getDimension()));
        error.segment(i, tsr->getDimension()) *= weights_[j];

        i += tsr->getDimension();
    }

    world_->unlock();
}

void TSRSet::getJacobianWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                   Eigen::Ref<Eigen::MatrixXd> jacobian) const
{
    world_->lock();

    unsigned int i = 0;
    std::size_t n = world.size();
    for (std::size_t j = 0; j < tsrs_.size(); ++j)
    {
        const auto &tsr = tsrs_[j];
        tsr->getJacobianWorldState(world, jacobian.block(i, 0, tsr->getDimension(), n));
        jacobian.block(i, 0, tsr->getDimension(), n) *= weights_[j];

        i += tsr->getDimension();
    }

    world_->unlock();
}

double TSRSet::distanceWorld() const
{
    Eigen::VectorXd x(getDimension());
    getErrorWorld(x);
    return x.norm();
}

double TSRSet::distanceWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const
{
    Eigen::VectorXd x(getDimension());
    getErrorWorldState(world, x);
    return x.norm();
}

bool TSRSet::solveWorld()
{
    world_->lock();

    if (tsrs_.size() == 1)
    {
        bool r = tsrs_[0]->solveWorld();
        world_->unlock();
        return r;
    }

    auto sim = world_->getSim();

    bool r = true;
    for (const auto &skidx : skel_indices_)
    {
        auto skel = sim->getSkeleton(skidx);
        auto ik = skel->getIK(true);
        r &= ik->solveAndApply(true);
    }

    world_->unlock();
    return r;
}

bool TSRSet::solveWorldState(Eigen::Ref<Eigen::VectorXd> world)
{
    world_->lock();
    for (auto &tsr : tsrs_)
        tsr->setPositionsWorldState(world);

    bool r = solveWorld();

    for (auto &tsr : tsrs_)
        tsr->getPositionsWorldState(world);

    world_->unlock();
    return r;
}

bool TSRSet::solveGradientWorldState(Eigen::Ref<Eigen::VectorXd> world)
{
    unsigned int iter = 0;
    double norm = 0;
    Eigen::VectorXd f(getDimension());
    Eigen::MatrixXd j(getDimension(), world.size());

    const double squared_tolerance = tolerance_ * tolerance_;
    const Eigen::VectorXd limit = Eigen::VectorXd::Constant(world.size(), limit_);

    world_->lock();
    getErrorWorldState(world, f);

    while ((norm = f.norm()) > squared_tolerance and iter++ < maxIter_)
    {
        getJacobianWorldState(world, j);
        if (qr_)
            world -= (step_ * j.colPivHouseholderQr().solve(f)).cwiseMin(limit).cwiseMax(-limit);
        else
        {
            if (damped_)
            {
                auto svd = j.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
                auto lr = svd.rank();
                const auto &u = svd.matrixU().leftCols(lr);
                const auto &v = svd.matrixV().leftCols(lr);
                const auto &s = svd.singularValues().head(lr);
                const auto &d = Eigen::VectorXd::Constant(lr, damping_);

                const auto &damped = s.cwiseQuotient(s.cwiseProduct(s) + d.cwiseProduct(d));

                Eigen::MatrixXd tmp;
                tmp.noalias() = u.adjoint() * f;
                tmp = damped.asDiagonal().inverse() * tmp;
                auto step = v * tmp;

                world -= (step_ * step).cwiseMin(limit).cwiseMax(-limit);
            }
            else
                world -= (step_ * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f))
                             .cwiseMin(limit)
                             .cwiseMax(-limit);
        }

        enforceBoundsWorld(world);

        getErrorWorldState(world, f);
    }

    world_->forceUpdate();
    world_->unlock();

    return norm < squared_tolerance;
}

void TSRSet::updateSolver()
{
    auto sim = world_->getSim();
    for (const auto &skidx : skel_indices_)
    {
        auto skel = sim->getSkeleton(skidx);
        auto ik = skel->getIK(true);

        auto sv = ik->getSolver();
        sv->setNumMaxIterations(maxIter_);
        sv->setTolerance(tolerance_);
    }
}

std::size_t TSRSet::getMaxIterations() const
{
    return maxIter_;
}

double TSRSet::getTolerance() const
{
    return tolerance_;
}

void TSRSet::initialize()
{
    skel_indices_.clear();
    for (auto &tsr : tsrs_)
    {
        tsr->initialize();
        skel_indices_.emplace(tsr->getSkeletonIndex());
    }

    updateSolver();
    RBX_INFO("TSRSet: Initialized %d TSRs!", tsrs_.size());
}

const std::vector<std::pair<std::size_t, std::size_t>> &TSRSet::getWorldIndices() const
{
    return tsrs_[0]->getWorldIndices();
}

void TSRSet::getPositionsWorldState(Eigen::Ref<Eigen::VectorXd> world) const
{
    const auto &wi = getWorldIndices();
    for (std::size_t i = 0; i < wi.size(); ++i)
    {
        const auto &wii = wi[i];
        world[i] = world_->getSim()->getSkeleton(wii.first)->getDof(wii.second)->getPosition();
    }
}

void TSRSet::computeLimits()
{
    const auto &wi = getWorldIndices();
    std::size_t n = wi.size();

    upper_ = Eigen::VectorXd::Zero(n);
    lower_ = Eigen::VectorXd::Zero(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        const auto &wii = wi[i];
        const auto &dof = world_->getSim()->getSkeleton(wii.first)->getDof(wii.second);
        // if (dof->isCyclic())
        // {
        //     lower_[i] = -constants::pi;
        //     upper_[i] = constants::pi;
        // }
        // else
        // {
        auto limits = dof->getPositionLimits();
        lower_[i] = limits.first;
        upper_[i] = limits.second;
        // }
    }
}

void TSRSet::setMaxIterations(std::size_t iterations)
{
    maxIter_ = iterations;
}

void TSRSet::setTolerance(double tolerance)
{
    tolerance_ = tolerance;
}

void TSRSet::setWorldUpperLimits(const Eigen::Ref<const Eigen::VectorXd> &upper)
{
    upper_ = upper;
}

void TSRSet::setWorldLowerLimits(const Eigen::Ref<const Eigen::VectorXd> &lower)
{
    lower_ = lower;
}

void TSRSet::enforceBoundsWorld(Eigen::Ref<Eigen::VectorXd> world) const
{
    if (upper_.size())
        world = world.cwiseMin(upper_);
    if (lower_.size())
        world = world.cwiseMax(lower_);
}

void TSRSet::print(std::ostream &out) const
{
    out << "TSRSet --------------------" << std::endl;
    for (const auto &tsr : tsrs_)
        tsr->getSpecification().print(out);
    out << "---------------------------" << std::endl;
}

void TSRSet::setStep(double step)
{
    step_ = step;
}

double TSRSet::getStep() const
{
    return step_;
}

void TSRSet::useSVD()
{
    qr_ = false;
}

void TSRSet::useQR()
{
    qr_ = true;
}

void TSRSet::setLimit(double limit)
{
    limit_ = limit;
}

double TSRSet::getLimit() const
{
    return limit_;
}

void TSRSet::setDamping(double damping)
{
    damping_ = damping;
}

double TSRSet::getDamping() const
{
    return damping_;
}

void TSRSet::useDamping(bool damping)
{
    damped_ = damping;
}

///
/// TSRConstraint
///

TSRConstraint::TSRConstraint(const StateSpacePtr &space, const TSRPtr &tsr)
  : TSRConstraint(space, std::make_shared<TSRSet>(space->getWorld(), tsr))
{
}

TSRConstraint::TSRConstraint(const StateSpacePtr &space, const std::vector<TSRPtr> &tsrs)
  : TSRConstraint(space, std::make_shared<TSRSet>(space->getWorld(), tsrs))
{
}

TSRConstraint::TSRConstraint(const StateSpacePtr &space, const TSRSetPtr &tsr)
  : ompl::base::Constraint(space->getDimension(), tsr->getDimension(), tsr->getTolerance())
  , space_(space)
  , tsr_(tsr)
{
    tsr_->useWorldIndices(space->getIndices());
    tsr_->setWorldIndices(space->getIndices());
    tsr_->setWorld(space->getWorld());

    tsr_->setWorldLowerLimits(space->getLowerBound());
    tsr_->setWorldUpperLimits(space->getUpperBound());
    tsr_->initialize();
}

void TSRConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::VectorXd> out) const
{
    tsr_->getErrorWorldState(x, out);
}

void TSRConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::MatrixXd> out) const
{
    tsr_->getJacobianWorldState(x, out);
}

bool TSRConstraint::project(Eigen::Ref<Eigen::VectorXd> x) const
{
    space_->setWorldState(space_->getWorld(), x);

    bool r = false;
    if (tsr_->numTSRs() == 1 or not options.use_gradient)
        r = tsr_->solveWorldState(x);

    else if (options.use_gradient)
        r = tsr_->solveGradientWorldState(x);

    return r;
}

TSRSetPtr TSRConstraint::getSet()
{
    return tsr_;
}
