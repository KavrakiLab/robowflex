/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/planning.h>

using namespace robowflex::darts;

PlanBuilder::PlanBuilder(WorldPtr world) : rspace(std::make_shared<StateSpace>(world)), world_(world)
{
}

void PlanBuilder::addGroup(const std::string &skeleton, const std::string &name)
{
    rspace->addGroup(skeleton, name);
}

void PlanBuilder::addConstraint(const TSRPtr &tsr)
{
    constraints_.emplace_back(tsr);
}

void PlanBuilder::setStartConfigurationFromWorld()
{
    start_ = Eigen::VectorXd::Zero(rspace->getDimension());
    rspace->getWorldState(world_, start_);
}

void PlanBuilder::setStartConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    start_ = q;
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
    goal_ = Eigen::VectorXd::Zero(rspace->getDimension());
    rspace->getWorldState(world_, goal_);
}

void PlanBuilder::setGoalConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    goal_ = q;
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
    if (constraints_.empty())
        initializeUnconstrained();
    else
        initializeConstrained();
}

void PlanBuilder::setup()
{
    ompl::base::ScopedState<> start_state(space);
    getState(start_state.get())->data = start_;

    ompl::base::ScopedState<> goal_state(space);
    getState(goal_state.get())->data = goal_;

    ss->setStartAndGoalStates(start_state, goal_state);
    ss->setup();
}

void PlanBuilder::initializeConstrained()
{
    if (constraints_.size() == 1)
        constraint_ = std::make_shared<darts::TSRConstraint>(rspace, constraints_[0]);
    else
        constraint_ = std::make_shared<darts::TSRCompositeConstraint>(rspace, constraints_);

    auto pss = std::make_shared<ompl::base::ProjectedStateSpace>(rspace, constraint_);
    space = pss;

    ss = std::make_shared<ompl::geometric::SimpleSetup>(space);
    setStateValidityChecker();

    info = ss->getSpaceInformation();

    pss->setSpaceInformation(info.get());
    pss->setDelta(0.05);
    pss->setLambda(2);
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
    if (constraint_)
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

        do
        {
            sampler->sampleUniform(state);
            space->enforceBounds(state);
        } while (not info->isValid(state));

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
            rspace->setWorldState(world_, as);

            return not world_->inCollision() and  //
                   ((constraint_) ? constraint_->isSatisfied(state) : true);
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
        rspace->setWorldState(world_, getState(path.getStates()[0]));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        for (const auto &state : path.getStates())
        {
            rspace->setWorldState(world_, getState(state));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
