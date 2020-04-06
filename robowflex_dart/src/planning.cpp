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

TSRGoal::TSRGoal(const WorldPtr &world, const StateSpacePtr &space, const ompl::base::SpaceInformationPtr &si,
                 const std::vector<TSRPtr> &tsrs, bool constrained)
  : ompl::base::GoalLazySamples(
        si, std::bind(&TSRGoal::sample, this, std::placeholders::_1, std::placeholders::_2), false)
  , world_(world->clone())
  , space_(space)
  , tsrs_(tsrs)
  , constrained_(constrained)
{
    // remap tsr structures - TODO copy tsrs
    for (auto &tsr : tsrs_)
    {
        auto name = tsr->getStructure()->getName();
        StructurePtr structure = world_->getRobot(name);
        if (not structure)
            structure = world_->getStructure(name);

        if (structure)
        {
            tsr->setStructure(structure);
            structures_.emplace(structure);
        }
        else
            throw std::runtime_error("Hey!");
    }
}

TSRGoal::TSRGoal(const PlanBuilder &builder, const std::vector<TSRPtr> &goal_tsrs)
  : TSRGoal(builder.world, builder.rspace, builder.info,
            [&] {
                std::vector<TSRPtr> tsrs = builder.constraints;
                tsrs.insert(tsrs.end(), goal_tsrs.begin(), goal_tsrs.end());
                return tsrs;
            }(),
            not builder.constraints.empty())
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, TSRPtr goal_tsr)
  : TSRGoal(builder, std::vector<TSRPtr>{goal_tsr})
{
}

bool TSRGoal::sample(const ompl::base::GoalLazySamples *gls, ompl::base::State *state)
{
    StateSpace::StateType *as;
    if (constrained_)
        as = state->as<ompl::base::ConstrainedStateSpace::StateType>()
                 ->getState()
                 ->as<darts::StateSpace::StateType>();
    else
        as = state->as<darts::StateSpace::StateType>();

    space_->setWorldState(world_, as);

    bool r = true;
    for (auto &structure : structures_)
        r &= structure->solveIK();

    space_->getWorldState(world_, as);
    return r;
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

    ompl::base::ScopedState<> goal_state(space);
    getState(goal_state.get())->data = goal;

    ss->setStartAndGoalStates(start_state, goal_state);
    ss->setup();
}

void PlanBuilder::initializeConstrained()
{
    if (constraints.size() == 1)
        constraint = std::make_shared<darts::TSRConstraint>(rspace, constraints[0]);
    else
        constraint = std::make_shared<darts::TSRCompositeConstraint>(rspace, constraints);

    auto pss = std::make_shared<ompl::base::ProjectedStateSpace>(rspace, constraint);
    space = pss;
    info = std::make_shared<ompl::base::ConstrainedSpaceInformation>(pss);

    ss = std::make_shared<ompl::geometric::SimpleSetup>(info);
    setStateValidityChecker();

    // pss->setDelta(0.05);
    pss->setDelta(0.1);
    pss->setLambda(1.5);
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
            ->as<darts::StateSpace::StateType>();
    else
        return state->as<darts::StateSpace::StateType>();
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
            rspace->setWorldState(world, as);

            // return not world->inCollision() and  //
            //        ((constraint) ? constraint->isSatisfied(state) : true);
            return not world->inCollision();
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
