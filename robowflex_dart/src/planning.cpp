/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <robowflex_library/tf.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/planning.h>

using namespace robowflex::darts;

///
/// WorldValidityChecker
///

WorldValidityChecker::WorldValidityChecker(const ompl::base::SpaceInformationPtr &si, std::size_t n)
  : ompl::base::StateValidityChecker(si)
  , constrained_(std::dynamic_pointer_cast<ompl::base::ConstrainedSpaceInformation>(si))
  , space_((constrained_ ? si_->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getSpace() :
                           si_->getStateSpace())
               ->as<StateSpace>())
  , world_(space_->getWorld())
{
    for (std::size_t i = 0; i < n; ++i)
        worlds_.emplace(world_->clone(std::to_string(i)));
}

bool WorldValidityChecker::isValid(const ompl::base::State *state) const
{
    auto world = getWorld();

    auto as = getStateConst(state);

    world->lock();
    space_->setWorldState(world, as);
    bool r = not world->inCollision();
    world->unlock();

    addWorld(world);

    return r;
}

const StateSpace::StateType *WorldValidityChecker::getStateConst(const ompl::base::State *state) const
{
    if (constrained_)
        return state->as<ompl::base::ConstrainedStateSpace::StateType>()
            ->getState()
            ->as<StateSpace::StateType>();
    else
        return state->as<StateSpace::StateType>();
}

WorldPtr WorldValidityChecker::getWorld() const
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return not worlds_.empty(); });

    WorldPtr world = worlds_.front();
    worlds_.pop();

    return world;
}

void WorldValidityChecker::addWorld(const WorldPtr &world) const
{
    std::unique_lock<std::mutex> lock(mutex_);
    worlds_.emplace(world);
    cv_.notify_one();
}

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

TSRGoal::TSRGoal(const PlanBuilder &builder, TSRPtr tsr) : TSRGoal(builder, std::vector<TSRPtr>{tsr})
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
    else
        return state->as<StateSpace::StateType>();
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

TSRPtr PlanBuilder::TSRfromPositionConstraint(const std::string &robot_name,
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

TSRPtr PlanBuilder::TSRfromOrientationConstraint(const std::string &robot_name,
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
        addConstraint(TSRfromPositionConstraint(robot_name, constraint));
    for (const auto &constraint : msg.path_constraints.orientation_constraints)
        addConstraint(TSRfromOrientationConstraint(robot_name, constraint));
}

void PlanBuilder::getGoalFromMessage(const std::string &robot_name, const moveit_msgs::MotionPlanRequest &msg)
{
    // TODO get other goals as well
    std::vector<TSRPtr> tsrs;
    for (const auto &constraint : msg.goal_constraints[0].position_constraints)
        tsrs.emplace_back(TSRfromPositionConstraint(robot_name, constraint));
    for (const auto &constraint : msg.goal_constraints[0].orientation_constraints)
        tsrs.emplace_back(TSRfromOrientationConstraint(robot_name, constraint));

    setGoalTSR(tsrs);
}

void PlanBuilder::fromMessage(const std::string &robot_name, const moveit_msgs::MotionPlanRequest &msg)
{
    getWorkspaceBoundsFromMessage(msg);
    getGroupFromMessage(robot_name, msg);
    getStartFromMessage(robot_name, msg);
    getPathConstraintsFromMessage(robot_name, msg);
    getGoalFromMessage(robot_name, msg);
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

void PlanBuilder::setGoalTSR(const TSRPtr &tsr)
{
    setGoalTSR(std::vector<TSRPtr>{tsr});
}

void PlanBuilder::setGoalTSR(const std::vector<TSRPtr> &tsrs)
{
    goal_constraints = tsrs;
}

void PlanBuilder::sampleGoalConfiguration()
{
    if (space)
        setGoalConfiguration(sampleState()->data);
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

    if (goal_constraints.empty())
    {
        ompl::base::ScopedState<> goal_state(space);
        getState(goal_state.get())->data = goal;
        ss->setGoalState(goal_state);
    }
    else
    {
        goal_tsr = std::make_shared<TSRGoal>(*this, goal_constraints);
        ss->setGoal(goal_tsr);
    }

    ss->setup();
}

void PlanBuilder::initializeConstrained()
{
    world->clearIKModules();

    constraint = std::make_shared<TSRConstraint>(rspace, path_constraints);

    std::cout << "Path Constraint" << std::endl;
    constraint->getSet()->print(std::cout);

    auto pss = std::make_shared<ompl::base::ProjectedStateSpace>(rspace, constraint);
    space = pss;
    info = std::make_shared<ompl::base::ConstrainedSpaceInformation>(pss);

    ss = std::make_shared<ompl::geometric::SimpleSetup>(info);
    setStateValidityChecker();

    // pss->setDelta(0.05);
    pss->setDelta(0.2);
    // pss->setDelta(0.1);
    // pss->setDelta(0.5);
    // pss->setLambda(2);
    // pss->setLambda(3);
    pss->setLambda(5);
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
        ss->setStateValidityChecker(std::make_shared<WorldValidityChecker>(info, 1));
        // ss->setStateValidityChecker([&](const ompl::base::State *state) -> bool {
        //     auto as = getStateConst(state);

        //     world->lock();
        //     rspace->setWorldState(world, as);
        //     bool r = not world->inCollision();
        //     world->unlock();
        //     return r;
        // });
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
