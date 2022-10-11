/* Author: Zachary Kingston */

#include <chrono>
#include <thread>
#include <utility>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <robowflex_dart/planning.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

using namespace robowflex::darts;

///
/// ConstraintExtractor
///
ConstraintExtractor::ConstraintExtractor(const ompl::base::SpaceInformationPtr &si)
{
    setSpaceInformation(si);
}

void ConstraintExtractor::setSpaceInformation(const ompl::base::SpaceInformationPtr &si)
{
    space_info_ = si;
    is_constrained_ = std::dynamic_pointer_cast<ompl::base::ConstrainedSpaceInformation>(si) != nullptr;
}

StateSpace::StateType *ConstraintExtractor::toState(ompl::base::State *state) const
{
    if (is_constrained_)
        return fromConstrainedState(state);

    return fromUnconstrainedState(state);
}

const StateSpace::StateType *ConstraintExtractor::toStateConst(const ompl::base::State *state) const
{
    return toState(const_cast<ompl::base::State *>(state));
}

StateSpace::StateType *ConstraintExtractor::fromConstrainedState(ompl::base::State *state) const
{
    return state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateSpace::StateType>();
}

const StateSpace::StateType *
ConstraintExtractor::fromConstrainedStateConst(const ompl::base::State *state) const
{
    return fromConstrainedState(const_cast<ompl::base::State *>(state));
}

StateSpace::StateType *ConstraintExtractor::fromUnconstrainedState(ompl::base::State *state) const
{
    return state->as<StateSpace::StateType>();
}

const StateSpace::StateType *
ConstraintExtractor::fromUnconstrainedStateConst(const ompl::base::State *state) const
{
    return fromUnconstrainedState(const_cast<ompl::base::State *>(state));
}

const StateSpace *ConstraintExtractor::getSpace() const
{
    return (is_constrained_ ?
                space_info_->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getSpace() :
                space_info_->getStateSpace())
        ->as<StateSpace>();
}

///
/// TSRGoal
///

TSRGoal::TSRGoal(const ompl::base::SpaceInformationPtr &si, const WorldPtr &world,
                 const std::vector<TSRPtr> &tsrs)
  : ompl::base::GoalLazySamples(
        si, std::bind(&TSRGoal::sample, this, std::placeholders::_1, std::placeholders::_2), false, 1e-3)
  , ConstraintExtractor(si)
  , world_(world->clone())
  , tsr_(std::make_shared<TSRSet>(world_, tsrs))
  // Have to allocate our own sampler from scratch since the constrained sampler might use the underlying
  // world used by the planner (e.g., in project)
  , sampler_(std::make_shared<StateSpace::StateSampler>(getSpace()))
{
    tsr_->useWorldIndices(getSpace()->getIndices());
    tsr_->setWorldIndices(getSpace()->getIndices());

    tsr_->setWorldLowerLimits(getSpace()->getLowerBound());
    tsr_->setWorldUpperLimits(getSpace()->getUpperBound());

    tsr_->initialize();
}

TSRGoal::TSRGoal(const ompl::base::SpaceInformationPtr &si, const WorldPtr &world, const TSRPtr tsr)
  : TSRGoal(si, world, std::vector<TSRPtr>{tsr})
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, TSRPtr tsr)
  : TSRGoal(builder, std::vector<TSRPtr>{std::move(tsr)})
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, const std::vector<TSRPtr> &tsrs)
  : TSRGoal(builder.info, builder.world, [&] {
      std::vector<TSRPtr> temp = builder.path_constraints;
      temp.insert(temp.end(), tsrs.begin(), tsrs.end());
      return temp;
  }())
{
}

TSRGoal::~TSRGoal()
{
    stopSampling();
}

bool TSRGoal::sample(const ompl::base::GoalLazySamples * /*gls*/, ompl::base::State *state)
{
    if (getStateCount() >= options.max_samples)
        return false;

    bool success = false;
    while (not terminateSamplingThread_ and not success)
    {
        auto &&as = toState(state);
        sampler_->sampleUniform(as);

        auto &&x = Eigen::Map<Eigen::VectorXd>(as->values, si_->getStateDimension());

        if (tsr_->numTSRs() == 1 or not options.use_gradient)
            success = tsr_->solveWorldState(x);

        else if (options.use_gradient)
            success = tsr_->solveGradientWorldState(x);

        success &= si_->satisfiesBounds(state);
        si_->enforceBounds(state);
    }

    return true;
}

double TSRGoal::distanceGoal(const ompl::base::State *state) const
{
    auto &&as = toStateConst(state);
    auto &&x = Eigen::Map<const Eigen::VectorXd>(as->values, si_->getStateDimension());
    return tsr_->distanceWorldState(x);
}

TSRSetPtr TSRGoal::getTSRSet()
{
    return tsr_;
}

///
/// JointRegionGoal
///

JointRegionGoal::JointRegionGoal(const PlanBuilder &builder, const Eigen::Ref<const Eigen::VectorXd> &state,
                                 double tolerance)
  : JointRegionGoal(builder,                                                     //
                    state - Eigen::VectorXd::Constant(state.size(), tolerance),  //
                    state + Eigen::VectorXd::Constant(state.size(), tolerance))
{
}

JointRegionGoal::JointRegionGoal(const PlanBuilder &builder, const Eigen::Ref<const Eigen::VectorXd> &lower,
                                 const Eigen::Ref<const Eigen::VectorXd> &upper)
  : ompl::base::GoalSampleableRegion(builder.info)
  , ConstraintExtractor(builder.info)
  , lower_(si_->allocState())
  , upper_(si_->allocState())
{
    if (lower.size() != upper.size())
        throw std::runtime_error("Bound size mismatch!");

    if (builder.space->getDimension() != lower.size())
        throw std::runtime_error("Bound size mismatch!");

    toState(lower_)->data = lower;
    toState(upper_)->data = upper;
}

JointRegionGoal::~JointRegionGoal()
{
    si_->freeState(lower_);
    si_->freeState(upper_);
}

void JointRegionGoal::sampleGoal(ompl::base::State *state) const
{
    auto &s = toState(state)->data;
    const auto &l = toStateConst(lower_)->data;
    const auto &u = toStateConst(upper_)->data;

    for (int i = 0; i < l.size(); ++i)
        s[i] = rng_.uniformReal(l[i], u[i]);
}

unsigned int JointRegionGoal::maxSampleCount() const
{
    return std::numeric_limits<unsigned int>::max();
}

double JointRegionGoal::distanceGoal(const ompl::base::State *state) const
{
    const auto &s = toStateConst(state)->data;
    const auto &l = toStateConst(lower_)->data;
    const auto &u = toStateConst(upper_)->data;

    const auto &ss = std::dynamic_pointer_cast<StateSpace>(si_->getStateSpace());

    double d = 0.;
    for (int i = 0; i < l.size(); ++i)
    {
        const auto &joint = ss->getJoint(i);
        const auto &sv = joint->getSpaceVarsConst(s);
        const auto &lv = joint->getSpaceVarsConst(l);
        const auto &uv = joint->getSpaceVarsConst(u);

        if (sv[0] < lv[0])
            d += joint->distance(sv, lv);
        if (sv[0] > uv[0])
            d += joint->distance(sv, uv);
    }

    return d;
}

///
/// PlanBuilder
///

PlanBuilder::PlanBuilder(WorldPtr world) : rspace(std::make_shared<StateSpace>(world)), world(world)
{
}

void PlanBuilder::addGroup(const std::string &skeleton, const std::string &name, std::size_t cyclic)
{
    rspace->addGroup(skeleton, name, cyclic);
}

void PlanBuilder::addConstraint(const TSRPtr &tsr)
{
    path_constraints.emplace_back(tsr);
}

void PlanBuilder::setStartConfigurationFromWorld()
{
    start = Eigen::VectorXd::Zero(rspace->getDimension());
    rspace->getWorldState(world, start);
}

void PlanBuilder::setStartConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    start = q;
}

void PlanBuilder::setStartConfiguration(const std::vector<double> &q)
{
    setStartConfiguration(Eigen::Map<const Eigen::VectorXd>(q.data(), rspace->getDimension()));
}

void PlanBuilder::sampleStartConfiguration()
{
    if (space)
    {
        auto *state = sampleState();
        setStartConfiguration(toState(state)->data);
        space->freeState(state);
    }
}

std::shared_ptr<ompl::base::GoalStates> PlanBuilder::getGoalConfigurationFromWorld()
{
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(rspace->getDimension());
    rspace->getWorldState(world, goal);
    return getGoalConfiguration(goal);
}

std::shared_ptr<ompl::base::GoalStates>
PlanBuilder::getGoalConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    if (not space)
        throw std::runtime_error("Builder must be initialized before creating goal!");

    ompl::base::ScopedState<> goal_state(space);
    toState(goal_state.get())->data = q;

    auto goal = std::make_shared<ompl::base::GoalStates>(info);
    goal->addState(goal_state);

    return goal;
}

std::shared_ptr<ompl::base::GoalStates> PlanBuilder::getGoalConfiguration(const std::vector<double> &q)
{
    return getGoalConfiguration(Eigen::Map<const Eigen::VectorXd>(q.data(), rspace->getDimension()));
}

TSRGoalPtr PlanBuilder::getGoalTSR(const TSRPtr &tsr)
{
    return getGoalTSR(std::vector<TSRPtr>{tsr});
}

TSRGoalPtr PlanBuilder::getGoalTSR(const std::vector<TSRPtr> &tsrs)
{
    if (not info)
        throw std::runtime_error("Builder must be initialized before creating goal!");

    return std::make_shared<TSRGoal>(*this, tsrs);
}

std::shared_ptr<ompl::base::GoalStates> PlanBuilder::sampleGoalConfiguration()
{
    if (space)
    {
        auto *state = sampleState();
        auto goal = getGoalConfiguration(toState(state)->data);
        space->freeState(state);

        return goal;
    }

    return nullptr;
}

void PlanBuilder::setGoal(const ompl::base::GoalPtr &goal)
{
    goal_ = goal;

    if (ss)
        ss->setGoal(goal_);
}

void PlanBuilder::initialize()
{
    if (path_constraints.empty())
        initializeUnconstrained();
    else
        initializeConstrained();

    setSpaceInformation(info);
}

void PlanBuilder::setup()
{
    ompl::base::ScopedState<> start_state(space);
    toState(start_state.get())->data = start;
    ss->setStartState(start_state);
    if (not goal_)
        throw std::runtime_error("No goal setup in PlanBuilder!");
    ss->setGoal(goal_);

    ss->setup();
}

void PlanBuilder::initializeConstrained()
{
    world->clearIKModules();
    rspace->setMetricSpace(false);

    rinfo = std::make_shared<ompl::base::SpaceInformation>(rspace);
    rinfo->setStateValidityChecker(getSVCUnconstrained());

    constraint = std::make_shared<TSRConstraint>(rspace, path_constraints);

    auto pss = std::make_shared<ompl::base::ProjectedStateSpace>(rspace, constraint);
    space = pss;
    info = std::make_shared<ompl::base::ConstrainedSpaceInformation>(pss);

    ss = std::make_shared<ompl::geometric::SimpleSetup>(info);
    setStateValidityChecker();

    pss->setDelta(options.constraints.delta);
    pss->setLambda(options.constraints.lambda);
}

void PlanBuilder::initializeUnconstrained()
{
    space = rspace;
    ss = std::make_shared<ompl::geometric::SimpleSetup>(space);
    setStateValidityChecker();

    rinfo = info = ss->getSpaceInformation();
}

ompl::base::State *PlanBuilder::sampleState() const
{
    if (space)
    {
        auto sampler = space->allocStateSampler();
        auto *state = space->allocState();

        bool valid = false;
        bool constrained = false;
        do
        {
            sampler->sampleUniform(state);
            space->enforceBounds(state);

            valid = info->isValid(state);
            if (constraint)
                constrained = constraint->isSatisfied(state);
            else
                constrained = true;
        } while (not valid or not constrained);

        return state;
    }

    return nullptr;
}

void PlanBuilder::setStateValidityChecker()
{
    if (ss)
    {
        // ss->setStateValidityChecker(std::make_shared<WorldValidityChecker>(info, 1));
        ss->setStateValidityChecker([&](const ompl::base::State *state) -> bool {
            const auto &as = toStateConst(state);

            world->lock();
            rspace->setWorldState(world, as);
            bool r = not world->inCollision();
            world->unlock();
            return r;
        });
    }
}

ompl::base::StateValidityCheckerFn PlanBuilder::getSVCUnconstrained()
{
    return [&](const ompl::base::State *state) -> bool {
        const auto &as = fromUnconstrainedStateConst(state);

        world->lock();
        rspace->setWorldState(world, as);
        bool r = not world->inCollision();
        world->unlock();
        return r;
    };
}

ompl::base::StateValidityCheckerFn PlanBuilder::getSVCConstrained()
{
    return [&](const ompl::base::State *state) -> bool {
        const auto &as = fromConstrainedStateConst(state);

        world->lock();
        rspace->setWorldState(world, as);
        bool r = not world->inCollision();
        world->unlock();
        return r;
    };
}

ompl::geometric::PathGeometric PlanBuilder::getSolutionPath(bool simplify, bool interpolate) const
{
    if (simplify)
        ss->simplifySolution();

    auto path = ss->getSolutionPath();
    if (interpolate)
        path.interpolate();

    return path;
}
