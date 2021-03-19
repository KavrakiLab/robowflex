/* Author: Zachary Kingston */

#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>

#include <robowflex_dart/planner.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/structure.h>
#include <robowflex_dart/world.h>

using namespace robowflex::darts;

DARTPlanner::DARTPlanner(const robowflex::RobotPtr &robot, const std::string &name)
  : robowflex::Planner(robot, name)
  , dart_robot_(std::make_shared<Robot>(robot))
  , world_(std::make_shared<World>())
{
    world_->addRobot(dart_robot_);
    setupPlanners();
    builder = std::make_shared<PlanBuilder>(world_);
}

void DARTPlanner::setupPlanners()
{
    // rrts
    planner_allocators_.emplace("rrt::RRT", makePlanner<ompl::geometric::RRT>());
    planner_allocators_.emplace("rrt::RRTConnect", makePlanner<ompl::geometric::RRTConnect>());
    planner_allocators_.emplace("rrt::RRTConnectIntermediate",
                                makePlanner<ompl::geometric::RRTConnect>(true));
    planner_allocators_.emplace("rrt::RRT*", makePlanner<ompl::geometric::RRTstar>());
    planner_allocators_.emplace("rrt::LazyRRT", makePlanner<ompl::geometric::LazyRRT>());

    // ests
    planner_allocators_.emplace("est::EST", makePlanner<ompl::geometric::EST>());
    planner_allocators_.emplace("est::BiEST", makePlanner<ompl::geometric::BiEST>());
    planner_allocators_.emplace("est::ProjEST", makePlanner<ompl::geometric::ProjEST>());

    // prms
    planner_allocators_.emplace("prm::PRM", makePlanner<ompl::geometric::PRM>());
    planner_allocators_.emplace("prm::PRM*", makePlanner<ompl::geometric::PRMstar>());
    planner_allocators_.emplace("prm::LazyPRM", makePlanner<ompl::geometric::LazyPRM>());
    planner_allocators_.emplace("prm::LazyPRM*", makePlanner<ompl::geometric::LazyPRMstar>());
    planner_allocators_.emplace("spars::SPARS", makePlanner<ompl::geometric::SPARS>());
    planner_allocators_.emplace("spars::SPARS2", makePlanner<ompl::geometric::SPARStwo>());

    // fmts
    planner_allocators_.emplace("fmt::FMT", makePlanner<ompl::geometric::FMT>());
    planner_allocators_.emplace("fmt::BFMT", makePlanner<ompl::geometric::BFMT>());

    // pdst / kpiece
    planner_allocators_.emplace("pdst::PDST", makePlanner<ompl::geometric::PDST>());
    planner_allocators_.emplace("kpiece::KPIECE", makePlanner<ompl::geometric::KPIECE1>());
    planner_allocators_.emplace("kpiece::BKPIECE", makePlanner<ompl::geometric::BKPIECE1>());
    planner_allocators_.emplace("kpiece::LBKPIECE", makePlanner<ompl::geometric::LBKPIECE1>());
}

void DARTPlanner::preRun(const robowflex::SceneConstPtr &scene,
                         const planning_interface::MotionPlanRequest &request)
{
    // remove old scene, if one exists
    if (dart_scene_)
        world_->removeStructure(dart_scene_);

    // convert scene representation
    scene_ = scene;
    dart_scene_ = std::make_shared<Structure>("scene", scene);
    world_->addStructure(dart_scene_);

    // setup planning request and get desired goal
    goal_ = builder->fromMessage(robot_->getName(), request);

    // find and setup planner
    auto it = planner_allocators_.find(request.planner_id);
    if (it == planner_allocators_.end())
        throw std::runtime_error("Invalid planner ID");

    builder->ss->setPlanner(it->second());
    builder->setup();
}

planning_interface::MotionPlanResponse DARTPlanner::plan(const robowflex::SceneConstPtr &scene,
                                                         const planning_interface::MotionPlanRequest &request)
{
    // if scene and preRun scene mismatch, recompute
    if (scene != scene_)
        preRun(scene, request);

    // start sampling if necessary
    auto gls = std::dynamic_pointer_cast<ompl::base::GoalLazySamples>(goal_);
    if (gls)
        gls->startSampling();

    // solve the plan, time the request
    auto start = std::chrono::steady_clock::now();
    ompl::base::PlannerStatus solved = builder->ss->solve(request.allowed_planning_time);
    auto end = std::chrono::steady_clock::now();

    auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // kill sampling
    if (gls)
        gls->stopSampling();

    // extract request
    planning_interface::MotionPlanResponse response;
    response.planning_time_ = 1e6 * (double)time;
    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        response.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
        response.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

    // extract trajectory
    auto path = builder->getSolutionPath();

    response.trajectory_ =
        std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModel(), request.group_name);

    for (const auto &s : path.getStates())
    {
        // set world state
        builder->rspace->setWorldState(world_, s);

        // copy over world state to moveit state
        auto ms = robot_->allocState();
        dart_robot_->setMoveItStateFromState(*ms);

        response.trajectory_->addSuffixWayPoint(ms, 0);
    }

    // compute time parameterization
    robowflex::Trajectory::computeTimeParameterization(*response.trajectory_);

    return response;
}

std::vector<std::string> DARTPlanner::getPlannerConfigs() const
{
    std::vector<std::string> configs;
    for (const auto &pair : planner_allocators_)
        configs.emplace_back(pair.first);

    return configs;
}
