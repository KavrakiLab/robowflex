/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/planning.h>

using namespace robowflex::darts;

///
/// TSRGoal
///

TSRGoal::TSRGoal(const ompl::base::ProblemDefinitionPtr pdef, const ompl::base::SpaceInformationPtr &si,
                 const WorldPtr &world, const std::vector<TSRPtr> &tsrs)
  : ompl::base::GoalLazySamples(
        si, std::bind(&TSRGoal::sample, this, std::placeholders::_1, std::placeholders::_2), false, 1e-3)
  , world_(world->clone())
  , tsr_(std::make_shared<TSRSet>(world_, tsrs))
  , constrained_(std::dynamic_pointer_cast<ompl::base::ConstrainedSpaceInformation>(si))
  // Have to allocate our own sampler from scratch since the constrained sampler might use the underlying
  // world used by the planner (e.g., in project)
  , sampler_(std::make_shared<StateSpace::StateSampler>(getSpace()))
  , pdef_(pdef)
{
    tsr_->setWorldLowerLimits(getSpace()->getLowerBound());
    tsr_->setWorldUpperLimits(getSpace()->getUpperBound());
    // tsr_->setMaxIterations(1000);
    // tsr_->addSuffix("_TSRGoal");
    tsr_->initialize();
}

TSRGoal::TSRGoal(const ompl::base::ProblemDefinitionPtr pdef, const ompl::base::SpaceInformationPtr &si,
                 const WorldPtr &world, const TSRPtr tsr)
  : TSRGoal(pdef, si, world, std::vector<TSRPtr>{tsr})
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, TSRPtr tsr) : TSRGoal(builder, std::vector<TSRPtr>{tsr})
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, const std::vector<TSRPtr> &tsrs)
  : TSRGoal(builder.ss->getProblemDefinition(), builder.info, builder.world, [&] {
      std::vector<TSRPtr> temp = builder.constraints;
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
    bool success = false;
    // std::size_t tries = attempts_;
    while (not success and not pdef_->hasSolution())
    {
        auto &&as = getState(state);
        sampler_->sampleUniform(as);

        auto &&x = Eigen::Map<Eigen::VectorXd>(as->values, si_->getStateDimension());

        if (tsr_->numTSRs() == 1)
            success = tsr_->solveWorldState(x);
        else
        {
            // tsr_->solveWorldState(x);
            success = tsr_->solveGradientWorldState(x);
        }

        // bool success2 = success;

        success &= si_->satisfiesBounds(state);
        // std::cout << "Sampled a goal with " << tsr_->distanceWorldState(x) << " to go! " << success2 << " "
        //           << success << std::endl;
    }

    // return getStateCount() < maxStateCount_;
    return true;
}

StateSpace::StateType *TSRGoal::getState(ompl::base::State *state) const
{
    if (constrained_)
        return state->as<ompl::base::ConstrainedStateSpace::StateType>()
            ->getState()
            ->as<StateSpace::StateType>();
    else
        return state->as<StateSpace::StateType>();
}

const StateSpace *TSRGoal::getSpace() const
{
    return (constrained_ ? si_->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getSpace() :
                           si_->getStateSpace())
        ->as<StateSpace>();
}

///
/// PlanBuilder
///

PlanBuilder::PlanBuilder(WorldPtr world) : rspace(std::make_shared<StateSpace>(world)), world(world)
{
}

void PlanBuilder::addGroup(const std::string &skeleton, const std::string &name)
{
    rspace->addGroup(skeleton, name, 2);
    // rspace->addGroup(skeleton, name);
}

void PlanBuilder::addConstraint(const TSRPtr &tsr)
{
    constraints.emplace_back(tsr);
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
        setStartConfiguration(sampleState()->data);
}

void PlanBuilder::setGoalConfigurationFromWorld()
{
    goal = Eigen::VectorXd::Zero(rspace->getDimension());
    rspace->getWorldState(world, goal);
}

void PlanBuilder::setGoalConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    goal = q;
}

void PlanBuilder::setGoalConfiguration(const std::vector<double> &q)
{
    setGoalConfiguration(Eigen::Map<const Eigen::VectorXd>(q.data(), rspace->getDimension()));
}

TSRGoalPtr PlanBuilder::setGoalTSR(const TSRPtr &tsr)
{
    auto goal = std::make_shared<TSRGoal>(*this, tsr);
    ss->setGoal(goal);

    return goal;
}

void PlanBuilder::sampleGoalConfiguration()
{
    if (space)
        setGoalConfiguration(sampleState()->data);
}

void PlanBuilder::initialize()
{
    if (constraints.empty())
        initializeUnconstrained();
    else
        initializeConstrained();
}

void PlanBuilder::setup()
{
    ompl::base::ScopedState<> start_state(space);
    getState(start_state.get())->data = start;
    ss->setStartState(start_state);

    auto tg = std::dynamic_pointer_cast<TSRGoal>(ss->getGoal());
    if (not tg)
    {
        ompl::base::ScopedState<> goal_state(space);
        getState(goal_state.get())->data = goal;
        ss->setGoalState(goal_state);
    }
    // else
    // tg->startSampling();

    ss->setup();
}

void PlanBuilder::initializeConstrained()
{
    world->clearIKModules();

    constraint = std::make_shared<TSRConstraint>(rspace, constraints);
    constraint->getSet()->setWorldIndices(rspace->getIndices());
    constraint->getSet()->setWorld(world);
    constraint->getSet()->setWorldLowerLimits(rspace->getLowerBound());
    constraint->getSet()->setWorldUpperLimits(rspace->getUpperBound());
    constraint->getSet()->initialize();

    auto pss = std::make_shared<ompl::base::ProjectedStateSpace>(rspace, constraint);
    space = pss;
    info = std::make_shared<ompl::base::ConstrainedSpaceInformation>(pss);

    ss = std::make_shared<ompl::geometric::SimpleSetup>(info);
    setStateValidityChecker();

    // pss->setDelta(0.05);
    // pss->setDelta(0.2);
    pss->setDelta(0.5);
    pss->setLambda(2);
    // pss->setLambda(3);
}

void PlanBuilder::initializeUnconstrained()
{
    space = rspace;
    ss = std::make_shared<ompl::geometric::SimpleSetup>(space);
    setStateValidityChecker();

    info = ss->getSpaceInformation();
}

StateSpace::StateType *PlanBuilder::getState(ompl::base::State *state) const
{
    if (constraint)
        return state->as<ompl::base::ConstrainedStateSpace::StateType>()
            ->getState()
            ->as<StateSpace::StateType>();
    else
        return state->as<StateSpace::StateType>();
}

const StateSpace::StateType *PlanBuilder::getStateConst(const ompl::base::State *state) const
{
    return getState(const_cast<ompl::base::State *>(state));
}

StateSpace::StateType *PlanBuilder::sampleState() const
{
    if (space)
    {
        auto sampler = space->allocStateSampler();
        auto state = space->allocState();

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

        return getState(state);
    }
    else
        return nullptr;
}

void PlanBuilder::setStateValidityChecker()
{
    if (ss)
    {
        ss->setStateValidityChecker([&](const ompl::base::State *state) -> bool {
            auto as = getStateConst(state);

            world->lock();
            rspace->setWorldState(world, as);

            // return not world->inCollision() and  //
            //        ((constraint) ? constraint->isSatisfied(state) : true);
            bool r = not world->inCollision();
            world->unlock();
            return r;
        });
    }
}

void PlanBuilder::animateSolutionInWorld(std::size_t times) const
{
    ss->simplifySolution();

    auto path = ss->getSolutionPath();
    path.interpolate(100);
    path.print(std::cout);

    std::size_t i = times;
    while ((times == 0) ? true : i--)
    {
        rspace->setWorldState(world, getState(path.getStates()[0]));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        const auto &states = path.getStates();
        for (std::size_t j = 0; j < states.size(); ++j)
        {
            if (not info->isValid(states[j]))
                std::cout << "State " << j << " is invalid!" << std::endl;
            rspace->setWorldState(world, getState(states[j]));
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
