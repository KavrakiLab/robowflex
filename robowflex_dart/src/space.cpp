/* Author: Zachary Kingston */

#include <ompl/base/spaces/RealVectorStateProjections.h>

#include <robowflex_library/constants.h>
#include <robowflex_library/log.h>

#include <robowflex_dart/space.h>
#include <robowflex_dart/world.h>

#include <utility>

namespace constants = robowflex::constants;
using namespace robowflex::darts;

///
/// StateSpace::StateSampler
///

StateSpace::StateSampler::StateSampler(const StateSpace *space)
  : ompl::base::RealVectorStateSampler(space), joints_(space->joints_)
{
}

void StateSpace::StateSampler::sampleUniform(ompl::base::State *state)
{
    auto *as = state->as<StateType>();

    for (const auto &joint : joints_)
        joint->sample(joint->getSpaceVars(as->data));
}

void StateSpace::StateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                                 double distance)
{
    auto *as = state->as<StateType>();
    const auto *an = near->as<StateType>();

    for (const auto &joint : joints_)
        joint->sampleNear(joint->getSpaceVars(as->data), joint->getSpaceVarsConst(an->data), distance);
}

///
/// StateSpace::StateType
///

StateSpace::StateType::StateType(std::size_t n) : data(n)
{
    values = data.data();
}

///
/// StateSpace::StateType
///

StateSpace::StateSpace(WorldPtr world) : ompl::base::RealVectorStateSpace(), world_(std::move(world))
{
}

bool StateSpace::isMetricSpace() const
{
    return metric_;
}

void StateSpace::addGroup(const std::string &name, const std::string &group, std::size_t cyclic)
{
    auto robot = world_->getRobot(name);
    if (not robot)
    {
        RBX_ERROR("Robot %1% does not exist in world.", name);
        throw std::runtime_error("Invalid robot");
    }

    auto joints = robot->getGroupJoints(group);
    addGroupFromJoints(group, joints, cyclic);
}

void StateSpace::addGroupFromJoints(const std::string &group_name,
                                    const std::vector<dart::dynamics::Joint *> &joints, std::size_t cyclic)
{
    group_joints_.emplace(group_name, std::vector<JointPtr>{});
    group_dimension_.emplace(group_name, 0);

    for (auto *joint : joints)
    {
        if (jointset_.find(joint) != jointset_.end())
        {
            addJointToGroup(group_name, getJoint(joint->getName()));
            continue;
        }

        jointset_.emplace(joint);

        const auto &type = joint->getType();
        if (type == "RevoluteJoint")
        {
            auto *revolute = static_cast<dart::dynamics::RevoluteJoint *>(joint);

            if (revolute->isCyclic(0))
            {
                if (cyclic)
                {
                    double low = -constants::pi * cyclic;
                    double high = constants::pi * cyclic;
                    addJoint(group_name, std::make_shared<RnJoint>(this, revolute, low, high));
                }
                else
                    addJoint(group_name, std::make_shared<SO2Joint>(this, revolute));
            }
            else
            {
                auto *dof = joint->getDof(0);
                auto limits = dof->getPositionLimits();
                addJoint(group_name, std::make_shared<RnJoint>(this, revolute, limits.first, limits.second));
            }
        }
        else if (type == "PrismaticJoint")
        {
            auto *prismatic = static_cast<dart::dynamics::PrismaticJoint *>(joint);
            auto *dof = joint->getDof(0);
            auto limits = dof->getPositionLimits();
            addJoint(group_name, std::make_shared<RnJoint>(this, prismatic, limits.first, limits.second));
        }
        else if (type == "PlanarJoint")
        {
            // Assume XY plane for now
            auto *planar = static_cast<dart::dynamics::PlanarJoint *>(joint);
            if (cyclic)
            {
                Eigen::Vector3d low;
                Eigen::Vector3d high;

                // XY limits
                for (std::size_t i = 0; i < 2; ++i)
                {
                    low[i] = world_->getWorkspaceLowConst()[i];
                    high[i] = world_->getWorkspaceHighConst()[i];
                }

                low[2] = -constants::pi * cyclic;
                high[2] = constants::pi * cyclic;

                addJoint(group_name, std::make_shared<RnJoint>(this, planar, 3, 0, low, high));
            }
            else
            {
                Eigen::Vector2d low;
                Eigen::Vector2d high;

                for (std::size_t i = 0; i < 2; ++i)
                {
                    low[i] = world_->getWorkspaceLowConst()[i];
                    high[i] = world_->getWorkspaceHighConst()[i];
                }

                addJoint(group_name, std::make_shared<RnJoint>(this, planar, 2, 0, low, high));
                addJoint(group_name, std::make_shared<SO2Joint>(this, planar, 2));
            }
        }
        else if (type == "FreeJoint")
        {
            auto *free = static_cast<dart::dynamics::FreeJoint *>(joint);

            if (cyclic)
            {
                double alow = -constants::pi * cyclic;
                double ahigh = constants::pi * cyclic;

                Eigen::Vector6d low;
                Eigen::Vector6d high;

                for (std::size_t i = 0; i < 3; ++i)
                {
                    low[i] = alow;
                    high[i] = ahigh;
                    low[3 + i] =
                        std::max({free->getPositionLowerLimit(3 + i), world_->getWorkspaceLowConst()[i]});
                    high[3 + i] =
                        std::min({free->getPositionUpperLimit(3 + i), world_->getWorkspaceHighConst()[i]});
                }
                auto j = std::make_shared<RnJoint>(this, free, 6, 0, low, high);
                addJoint(group_name, j);
            }
            else
            {
                Eigen::Vector3d low;
                Eigen::Vector3d high;

                for (std::size_t i = 0; i < 3; ++i)
                {
                    low[i] =
                        std::max({free->getPositionLowerLimit(3 + i), world_->getWorkspaceLowConst()[i]});
                    high[i] =
                        std::min({free->getPositionUpperLimit(3 + i), world_->getWorkspaceHighConst()[i]});
                }

                addJoint(group_name, std::make_shared<RnJoint>(this, free, 3, 3, low, high));
                addJoint(group_name, std::make_shared<SO3Joint>(this, free));
            }
        }
        else
            RBX_WARN("Unknown joint type %1%, skipping.", type);
    }

    registerDefaultProjection(
        std::make_shared<ompl::base::RealVectorRandomLinearProjectionEvaluator>(this, 2));
}

void StateSpace::addJoint(const std::string &group_name, const JointPtr &joint)
{
    joints_.emplace_back(joint);
    addJointToGroup(group_name, joint);

    if (not std::dynamic_pointer_cast<RnJoint>(joint))
        metric_ = false;
}

void StateSpace::addJointToGroup(const std::string &group_name, const JointPtr &joint)
{
    group_joints_[group_name].emplace_back(joint);
    group_dimension_[group_name] += joint->getDimension();
}

void StateSpace::setWorldState(WorldPtr world, const ompl::base::State *state) const
{
    const auto &as = state->as<StateType>();
    setWorldState(std::move(world), as->data);
}

void StateSpace::setWorldState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &x) const
{
    for (const auto &joint : joints_)
    {
        const auto &v = joint->getSpaceVarsConst(x);
        joint->setJointState(world, v);
    }

    world->forceUpdate();
}

void StateSpace::getWorldState(WorldPtr world, ompl::base::State *state) const
{
    auto *as = state->as<StateType>();
    getWorldState(std::move(world), as->data);
}

void StateSpace::getWorldState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> x) const
{
    for (const auto &joint : joints_)
    {
        const auto &v = joint->getSpaceVars(x);
        joint->getJointState(world, v);
    }
}

void StateSpace::setWorldGroupState(WorldPtr world, const std::string &group_name,
                                    const Eigen::Ref<const Eigen::VectorXd> &x) const
{
    const auto &joints = group_joints_.find(group_name)->second;

    std::size_t index = 0;
    for (const auto &joint : joints)
    {
        std::size_t n = joint->getDimension();
        joint->setJointState(world, x.segment(index, n));
        index += n;
    }

    world->forceUpdate();
}

void StateSpace::getWorldGroupState(WorldPtr world, const std::string &group_name,
                                    Eigen::Ref<Eigen::VectorXd> x) const
{
    const auto &joints = group_joints_.find(group_name)->second;

    std::size_t index = 0;
    for (const auto &joint : joints)
    {
        std::size_t n = joint->getDimension();
        joint->getJointState(world, x.segment(index, n));
        index += n;
    }
}

void StateSpace::enforceBounds(ompl::base::State *state) const
{
    const auto &as = state->as<StateType>();
    for (const auto &joint : joints_)
    {
        auto v = joint->getSpaceVars(as->data);
        joint->enforceBounds(v);
    }
}

bool StateSpace::satisfiesBounds(const ompl::base::State *state) const
{
    const auto &as = state->as<StateType>();
    for (const auto &joint : joints_)
    {
        const auto &v = joint->getSpaceVarsConst(as->data);
        if (not joint->satisfiesBounds(v))
            return false;
    }

    return true;
}

double StateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    const auto &as1 = state1->as<StateType>();
    const auto &as2 = state2->as<StateType>();

    double d = 0;
    for (const auto &joint : joints_)
    {
        const auto &v1 = joint->getSpaceVarsConst(as1->data);
        const auto &v2 = joint->getSpaceVarsConst(as2->data);

        d += joint->distance(v1, v2);
    }

    return d;
}

double StateSpace::getMaximumExtent() const
{
    double d = 0;
    for (const auto &joint : joints_)
        d += joint->getMaximumExtent();

    return d;
}

bool StateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    return distance(state1, state2) <= 1e-8;
}

void StateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                             ompl::base::State *state) const
{
    const auto &af = from->as<StateType>();
    const auto &at = to->as<StateType>();
    auto *as = state->as<StateType>();

    for (const auto &joint : joints_)
    {
        const auto &vf = joint->getSpaceVarsConst(af->data);
        const auto &vt = joint->getSpaceVarsConst(at->data);
        auto vs = joint->getSpaceVars(as->data);

        joint->interpolate(vf, vt, t, vs);
    }
}

ompl::base::StateSamplerPtr StateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<StateSampler>(this);
}

ompl::base::State *StateSpace::allocState() const
{
    return new StateType(dimension_);
}

void StateSpace::freeState(ompl::base::State *state) const
{
    auto *as = state->as<StateType>();
    delete as;
}

WorldPtr StateSpace::getWorld()
{
    return world_;
}

const WorldPtr &StateSpace::getWorldConst() const
{
    return world_;
}

std::vector<std::pair<std::size_t, std::size_t>> StateSpace::getIndices() const
{
    std::vector<std::pair<std::size_t, std::size_t>> indices;
    for (const auto &joint : joints_)
    {
        auto skelx = joint->getSkeletonIndex();
        auto add = joint->getIndices();
        for (const auto &index : add)
            indices.emplace_back(skelx, index);
    }

    return indices;
}

JointPtr StateSpace::getJoint(std::size_t index) const
{
    return joints_[index];
}

JointPtr StateSpace::getJoint(const std::string &name) const
{
    auto it = std::find_if(joints_.begin(), joints_.end(),
                           [&](const JointPtr &j) { return j->getJoint(world_)->getName() == name; });
    if (it != joints_.end())
        return *it;
    return nullptr;
}

const std::vector<JointPtr> &StateSpace::getJoints() const
{
    return joints_;
}

void StateSpace::getGroupState(const std::string &group, const ompl::base::State *state,
                               Eigen::Ref<Eigen::VectorXd> v) const
{
    const auto &joints = group_joints_.find(group)->second;
    const auto &as = state->as<StateType>();

    std::size_t index = 0;
    for (const auto &joint : joints)
    {
        std::size_t n = joint->getDimension();
        v.segment(index, n) = joint->getSpaceVarsConst(as->data);
        index += n;
    }
}

void StateSpace::setGroupState(const std::string &group, ompl::base::State *state,
                               const Eigen::Ref<const Eigen::VectorXd> &v) const
{
    const auto &joints = group_joints_.find(group)->second;
    auto *as = state->as<StateType>();

    std::size_t index = 0;
    for (const auto &joint : joints)
    {
        std::size_t n = joint->getDimension();
        joint->getSpaceVars(as->data) = v.segment(index, n);
        index += n;
    }
}

std::size_t StateSpace::getGroupDimension(const std::string &group) const
{
    return group_dimension_.find(group)->second;
}

std::vector<std::string> StateSpace::getGroups() const
{
    std::vector<std::string> groups;
    for (const auto &pair : group_joints_)
        groups.emplace_back(pair.first);

    return groups;
}

std::vector<std::string> StateSpace::getGroupDofNames(const std::string &group_name) const
{
    std::vector<std::string> dofs;

    const auto &joints = group_joints_.find(group_name)->second;
    for (const auto &joint : joints)
    {
        const auto &jdofs = joint->getDofs();
        dofs.insert(dofs.end(), jdofs.begin(), jdofs.end());
    }

    return dofs;
}

Eigen::VectorXd StateSpace::getLowerBound() const
{
    const auto &bounds = getBounds();
    return Eigen::Map<const Eigen::VectorXd>(bounds.low.data(), bounds.low.size());
}

Eigen::VectorXd StateSpace::getUpperBound() const
{
    const auto &bounds = getBounds();
    return Eigen::Map<const Eigen::VectorXd>(bounds.high.data(), bounds.high.size());
}

void StateSpace::setMetricSpace(bool metric)
{
    metric_ = metric;
}
