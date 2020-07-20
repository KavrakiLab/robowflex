/* Author: Zachary Kingston */

#include <chrono>
#include <thread>
#include <utility>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <robowflex_library/tf.h>

#include <robowflex_dart/planning.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

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
    tsr_->useWorldIndices(getSpace()->getIndices());
    tsr_->setWorldIndices(getSpace()->getIndices());

    tsr_->setWorldLowerLimits(getSpace()->getLowerBound());
    tsr_->setWorldUpperLimits(getSpace()->getUpperBound());

    tsr_->initialize();

    std::cout << "TSRGoal" << std::endl;
    tsr_->print(std::cout);
}

TSRGoal::TSRGoal(const ompl::base::ProblemDefinitionPtr pdef, const ompl::base::SpaceInformationPtr &si,
                 const WorldPtr &world, const TSRPtr tsr)
  : TSRGoal(pdef, si, world, std::vector<TSRPtr>{tsr})
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, TSRPtr tsr)
  : TSRGoal(builder, std::vector<TSRPtr>{std::move(tsr)})
{
}

TSRGoal::TSRGoal(const PlanBuilder &builder, const std::vector<TSRPtr> &tsrs)
  : TSRGoal(builder.ss->getProblemDefinition(), builder.info, builder.world, [&] {
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
    while (not success and not pdef_->hasSolution())
    {
        auto &&as = getState(state);
        sampler_->sampleUniform(as);

        auto &&x = Eigen::Map<Eigen::VectorXd>(as->values, si_->getStateDimension());

        if (tsr_->numTSRs() == 1 or not options.use_gradient)
            success = tsr_->solveWorldState(x);

        else if (options.use_gradient)
            success = tsr_->solveGradientWorldState(x);

        success &= si_->satisfiesBounds(state);
        si_->enforceBounds(state);
    }

    total_samples_++;
    return total_samples_ < options.max_samples;
}

double TSRGoal::distanceGoal(const ompl::base::State *state) const
{
    auto &&as = getStateConst(state);
    auto &&x = Eigen::Map<const Eigen::VectorXd>(as->values, si_->getStateDimension());
    return tsr_->distanceWorldState(x);
}

const StateSpace::StateType *TSRGoal::getStateConst(const ompl::base::State *state) const
{
    if (constrained_)
        return state->as<ompl::base::ConstrainedStateSpace::StateType>()
            ->getState()
            ->as<StateSpace::StateType>();

    return state->as<StateSpace::StateType>();
}

StateSpace::StateType *TSRGoal::getState(ompl::base::State *state) const
{
    if (constrained_)
        return state->as<ompl::base::ConstrainedStateSpace::StateType>()
            ->getState()
            ->as<StateSpace::StateType>();

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

void PlanBuilder::getWorkspaceBoundsFromMessage(const moveit_msgs::MotionPlanRequest &msg)
{
    const auto &mic = msg.workspace_parameters.min_corner;
    world->getWorkspaceLow()[0] = mic.x;
    world->getWorkspaceLow()[1] = mic.y;
    world->getWorkspaceLow()[2] = mic.z;

    const auto &mac = msg.workspace_parameters.max_corner;
    world->getWorkspaceHigh()[0] = mac.x;
    world->getWorkspaceHigh()[1] = mac.y;
    world->getWorkspaceHigh()[2] = mac.z;
}

void PlanBuilder::getGroupFromMessage(const std::string &robot_name,
                                      const moveit_msgs::MotionPlanRequest &msg)
{
    bool cyclic = false;
    if (not msg.path_constraints.position_constraints.empty() or
        not msg.path_constraints.orientation_constraints.empty())
        cyclic = true;

    addGroup(robot_name, msg.group_name, cyclic ? 2 : 0);
}

void PlanBuilder::getStartFromMessage(const std::string &robot_name,
                                      const moveit_msgs::MotionPlanRequest &msg)
{
    auto robot = world->getRobot(robot_name);
    for (std::size_t i = 0; i < msg.start_state.joint_state.name.size(); ++i)
    {
        std::string name = msg.start_state.joint_state.name[i];
        double value = msg.start_state.joint_state.position[i];

        robot->setJoint(name, value);
    }

    setStartConfigurationFromWorld();
}

TSRPtr PlanBuilder::fromPositionConstraint(const std::string &robot_name,
                                           const moveit_msgs::PositionConstraint &msg) const
{
    TSR::Specification spec;
    spec.setFrame(robot_name, msg.link_name);
    if (not msg.header.frame_id.empty() and msg.header.frame_id != "world")
        spec.setBase(robot_name, msg.header.frame_id);

    if (not msg.constraint_region.meshes.empty())
    {
        // TODO error, can't do this
        std::cerr << "Invalid Position Constraint" << std::endl;
        return nullptr;
    }

    if (msg.constraint_region.primitives.size() != 1)
    {
        // TODO error, can't do this
        std::cerr << "Invalid Position Constraint" << std::endl;
        return nullptr;
    }

    spec.setNoRotTolerance();

    // get pose
    spec.setPose(TF::poseMsgToEigen(msg.constraint_region.primitive_poses[0]));
    spec.setPosition(spec.getPosition() + TF::vectorMsgToEigen(msg.target_point_offset));

    auto &&primitive = msg.constraint_region.primitives[0];
    if (primitive.type == shape_msgs::SolidPrimitive::BOX)
    {
        spec.setXPosTolerance(primitive.dimensions[0]);
        spec.setYPosTolerance(primitive.dimensions[1]);
        spec.setZPosTolerance(primitive.dimensions[2]);
    }
    else if (primitive.type == shape_msgs::SolidPrimitive::SPHERE)
    {
        spec.setXPosTolerance(primitive.dimensions[0]);
        spec.setYPosTolerance(primitive.dimensions[0]);
        spec.setZPosTolerance(primitive.dimensions[0]);
    }

    // spec.print(std::cout);
    return std::make_shared<TSR>(world, spec);
}

TSRPtr PlanBuilder::fromOrientationConstraint(const std::string &robot_name,
                                              const moveit_msgs::OrientationConstraint &msg) const
{
    TSR::Specification spec;
    spec.setFrame(robot_name, msg.link_name);
    if (not msg.header.frame_id.empty() and msg.header.frame_id != "world")
        spec.setBase(robot_name, msg.header.frame_id);

    spec.setRotation(TF::quaternionMsgToEigen(msg.orientation));
    spec.setNoPosTolerance();
    spec.setXRotTolerance(msg.absolute_x_axis_tolerance);
    spec.setYRotTolerance(msg.absolute_y_axis_tolerance);
    spec.setZRotTolerance(msg.absolute_z_axis_tolerance);

    // spec.print(std::cout);
    return std::make_shared<TSR>(world, spec);
}

void PlanBuilder::getPathConstraintsFromMessage(const std::string &robot_name,
                                                const moveit_msgs::MotionPlanRequest &msg)
{
    for (const auto &constraint : msg.path_constraints.position_constraints)
        addConstraint(fromPositionConstraint(robot_name, constraint));
    for (const auto &constraint : msg.path_constraints.orientation_constraints)
        addConstraint(fromOrientationConstraint(robot_name, constraint));
}

ompl::base::GoalPtr PlanBuilder::getGoalFromMessage(const std::string &robot_name,
                                                    const moveit_msgs::MotionPlanRequest &msg)
{
    // TODO get other goals as well
    std::vector<TSRPtr> tsrs;
    for (const auto &constraint : msg.goal_constraints[0].position_constraints)
        tsrs.emplace_back(fromPositionConstraint(robot_name, constraint));
    for (const auto &constraint : msg.goal_constraints[0].orientation_constraints)
        tsrs.emplace_back(fromOrientationConstraint(robot_name, constraint));

    return getGoalTSR(tsrs);
}

ompl::base::GoalPtr PlanBuilder::fromMessage(const std::string &robot_name,
                                             const moveit_msgs::MotionPlanRequest &msg)
{
    getWorkspaceBoundsFromMessage(msg);
    getGroupFromMessage(robot_name, msg);
    getStartFromMessage(robot_name, msg);
    getPathConstraintsFromMessage(robot_name, msg);
    initialize();

    auto goal = getGoalFromMessage(robot_name, msg);
    setGoal(goal);

    return goal;
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
        auto state = sampleState();
        setStartConfiguration(getState(state)->data);
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
    getState(goal_state.get())->data = q;

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
        auto state = sampleState();
        auto goal = getGoalConfiguration(getState(state)->data);
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
}

void PlanBuilder::setup()
{
    ompl::base::ScopedState<> start_state(space);
    getState(start_state.get())->data = start;
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

    std::cout << "Path Constraint" << std::endl;
    constraint->getSet()->print(std::cout);

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

StateSpace::StateType *PlanBuilder::getState(ompl::base::State *state) const
{
    if (constraint)
        return getFromConstrainedState(state);

    return getFromUnconstrainedState(state);
}

const StateSpace::StateType *PlanBuilder::getStateConst(const ompl::base::State *state) const
{
    return getState(const_cast<ompl::base::State *>(state));
}

StateSpace::StateType *PlanBuilder::getFromConstrainedState(ompl::base::State *state) const
{
    return state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateSpace::StateType>();
}

const StateSpace::StateType *PlanBuilder::getFromConstrainedStateConst(const ompl::base::State *state) const
{
    return getFromConstrainedState(const_cast<ompl::base::State *>(state));
}

StateSpace::StateType *PlanBuilder::getFromUnconstrainedState(ompl::base::State *state) const
{
    return state->as<StateSpace::StateType>();
}

const StateSpace::StateType *PlanBuilder::getFromUnconstrainedStateConst(const ompl::base::State *state) const
{
    return getFromUnconstrainedState(const_cast<ompl::base::State *>(state));
}

ompl::base::State *PlanBuilder::sampleState() const
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
            auto as = getStateConst(state);

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
        auto as = getFromUnconstrainedStateConst(state);

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
        auto as = getFromConstrainedStateConst(state);

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
