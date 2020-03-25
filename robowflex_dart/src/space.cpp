/* Author: Zachary Kingston */

#include <robowflex_dart/space.h>
#include <robowflex_dart/world.h>

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
    auto as = state->as<StateType>();

    for (const auto &joint : joints_)
        joint->sample(joint->getSpaceVars(as->data));
}

void StateSpace::StateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                                 double distance)
{
    auto as = state->as<StateType>();
    auto an = near->as<StateType>();

    for (const auto &joint : joints_)
        joint->sampleNear(joint->getSpaceVars(as->data), joint->getSpaceVarsConst(an->data), distance);
}

///
/// StateSpace::StateType
///

StateSpace::StateType::StateType(unsigned int n) : data(n)
{
    values = data.data();
}

///
/// StateSpace::StateType
///

StateSpace::StateSpace(WorldPtr world) : ompl::base::RealVectorStateSpace(), world_(world)
{
}

bool StateSpace::isMetricSpace() const
{
    return false;
}

void StateSpace::addGroup(const std::string &name, const std::string &group, unsigned int cyclic)
{
    auto robot = world_->getRobot(name);
    if (not robot)
    {
        // SE3EZ_ERROR("Robot %1% does not exist in world.", name);
        throw std::runtime_error("Invalid robot");
    }

    auto joints = robot->getGroupJoints(group);

    for (auto joint : joints)
    {
        if (jointset_.find(joint) != jointset_.end())
        {
            // SE3EZ_INFORM("Joint %1% already being planned, skipping...", joint->getName());
            continue;
        }
        else
            jointset_.emplace(joint);

        const auto &type = joint->getType();
        JointPtr add = nullptr;
        if (type == "RevoluteJoint")
        {
            auto revolute = static_cast<dart::dynamics::RevoluteJoint *>(joint);

            if (revolute->isCyclic(0))
            {
                if (cyclic)
                {
                    double low = -constants::pi * cyclic;
                    double high = constants::pi * cyclic;
                    joints_.emplace_back(std::make_shared<RnJoint>(this, revolute, low, high));
                }
                else
                    joints_.emplace_back(std::make_shared<SO2Joint>(this, revolute));
            }
            else
            {
                auto dof = joint->getDof(0);
                auto limits = dof->getPositionLimits();
                joints_.emplace_back(std::make_shared<RnJoint>(this, revolute, limits.first, limits.second));
            }
        }
        else if (type == "PrismaticJoint")
        {
            auto prismatic = static_cast<dart::dynamics::PrismaticJoint *>(joint);
            auto dof = joint->getDof(0);
            auto limits = dof->getPositionLimits();
            joints_.emplace_back(std::make_shared<RnJoint>(this, prismatic, limits.first, limits.second));
        }
        else if (type == "FreeJoint")
        {
            auto free = static_cast<dart::dynamics::FreeJoint *>(joint);

            if (cyclic)
            {
                double alow = -constants::pi * cyclic;
                double ahigh = constants::pi * cyclic;

                Eigen::Vector6d low;
                Eigen::Vector6d high;

                for (unsigned int i = 0; i < 3; ++i)
                {
                    low[i] = alow;
                    high[i] = ahigh;
                    low[3 + i] =
                        std::max({free->getPositionLowerLimit(3 + i), world_->getWorkspaceLowConst()[i]});
                    high[3 + i] =
                        std::min({free->getPositionUpperLimit(3 + i), world_->getWorkspaceHighConst()[i]});
                }

                joints_.emplace_back(std::make_shared<RnJoint>(this, free, 6, 0, low, high));
            }
            else
            {
                Eigen::Vector3d low;
                Eigen::Vector3d high;

                for (unsigned int i = 0; i < 3; ++i)
                {
                    low[i] =
                        std::max({free->getPositionLowerLimit(3 + i), world_->getWorkspaceLowConst()[i]});
                    high[i] =
                        std::min({free->getPositionUpperLimit(3 + i), world_->getWorkspaceHighConst()[i]});
                }

                joints_.emplace_back(std::make_shared<RnJoint>(this, free, 3, 3, low, high));
                joints_.emplace_back(std::make_shared<SO3Joint>(this, free));
            }
        }
        // else
        //     SE3EZ_WARN("Unknown joint type %1%, skipping.", type);
    }
}

void StateSpace::setWorldState(WorldPtr world, const ompl::base::State *state)
{
    const auto &as = state->as<StateType>();

    for (const auto &joint : joints_)
    {
        const auto &v = joint->getSpaceVarsConst(as->data);
        joint->setJoint(world, v);
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
    auto as = state->as<StateType>();

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
    auto as = state->as<StateType>();
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
