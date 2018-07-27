/* Author: Bryce Willey */

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/sbl/SBL.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

#include <robowflex_tesseract/tesseract_planners.h>
#include <robowflex_tesseract/conversions.h>

#include <tesseract_planning/ompl/continuous_motion_validator.h>
#include <moveit/robot_state/conversions.h>

using namespace robowflex::hypercube;

namespace
{
    template <typename T>
    static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr &si,
                                                  const std::string &new_name,
                                                  const std::map<std::string, std::string> &config)
    {
        ompl::base::PlannerPtr planner(new T(si));
        if (not new_name.empty())
            planner->setName(new_name);

        planner->params().setParams(config, true);
        planner->setup();
        return planner;
    }
}  // namespace

Settings::Settings()
  : simplify_solutions(true), longest_valid_segment_fraction(0.01), use_continuous_validator(false)
{
}

void OMPLChainPlanner::registerDefaultPlanners()
{
    registerPlannerAllocator("geometric::RRT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::RRT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::RRTConnect",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::RRTConnect>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::LazyRRT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::LazyRRT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::RRTstar",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::RRTstar>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::InformedRRTstar",                          //
                             std::bind(&allocatePlanner<ompl::geometric::RRTstar>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::TRRT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::TRRT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::BiTRRT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::BiTRRT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::LBTRRT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::LBTRRT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::LazyLBTRRT",                              //
                             std::bind(&allocatePlanner<ompl::geometric::LBTRRT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    registerPlannerAllocator("geometric::EST",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::EST>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::BiEST",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::BiEST>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::ProjEST",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::ProjEST>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::SBL",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::SBL>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    registerPlannerAllocator("geometric::KPIECE",                                   //
                             std::bind(&allocatePlanner<ompl::geometric::KPIECE1>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::BKPIECE",                                   //
                             std::bind(&allocatePlanner<ompl::geometric::BKPIECE1>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::LBKPIECE",                                   //
                             std::bind(&allocatePlanner<ompl::geometric::LBKPIECE1>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    registerPlannerAllocator("geometric::PRM",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::PRM>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::PRMstar",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::PRMstar>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::LazyPRM",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::LazyPRM>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::LazyPRMstar",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::LazyPRMstar>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::SPARS",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::SPARS>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::SPARStwo",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::SPARStwo>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    registerPlannerAllocator("geometric::FMT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::FMT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::BFMT",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::BFMT>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::BITstar",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::BITstar>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    registerPlannerAllocator("geometric::PDST",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::PDST>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    registerPlannerAllocator("geometric::STRIDE",                                  //
                             std::bind(&allocatePlanner<ompl::geometric::STRIDE>,  //
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

OMPLChainPlanner::OMPLChainPlanner(const RobotPtr &robot, const std::string &name) : Planner(robot, name)
{
}

bool OMPLChainPlanner::initialize(const std::string &config_file, const Settings &settings)
{
    // TODO: Actually do something with the config file, for now it seems like we have to duplicate all of
    // MoveIt's ompl_interface.
    // if (not OMPL::loadOMPLConfig(handler_, config_file, configs_))
    //    return false;

    registerDefaultPlanners();

    settings_.simplify_solutions = settings.simplify_solutions;
    settings_.longest_valid_segment_fraction = settings.longest_valid_segment_fraction;
    settings_.use_continuous_validator = settings.use_continuous_validator;

    return true;
}

void OMPLChainPlanner::registerPlannerAllocator(const std::string &planner_id,
                                                const ConfiguredPlannerAllocator &pa)
{
    known_planners_[planner_id] = pa;
}

planning_interface::MotionPlanResponse
OMPLChainPlanner::plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse res;

    tesseract::tesseract_ros::KDLEnvPtr env = constructTesseractEnv(scene, getRobot());

    // Make the OMPL Context.
    chain_interface_ =
        std::make_shared<tesseract::tesseract_planning::ChainOmplInterface>(env, request.group_name);

    // Set the Motion Validator in chain_interface_.
    if (settings_.use_continuous_validator)
    {
        auto mv = std::make_shared<tesseract::tesseract_planning::ContinuousMotionValidator>(
            chain_interface_->spaceInformation(), env, request.group_name);
        chain_interface_->setMotionValidator(mv);
    }
    chain_interface_->spaceInformation()->setup();
    std::map<std::string, std::string> config;
    config["longest_valid_segment_fraction"] = std::to_string(settings_.longest_valid_segment_fraction);
    chain_interface_->spaceInformation()->params().setParams(config, true);
    chain_interface_->spaceInformation()->setup();

    ompl::base::PlannerPtr planner;
    auto it = known_planners_.find(request.planner_id);
    if (it != known_planners_.end())
    {
        std::map<std::string, std::string> config;  // TODO: get the actual version of this.
        planner = it->second(chain_interface_->spaceInformation(), "tesseract_planner?", config);
    }
    else
    {
        ROS_ERROR("Unknown planner: '%s'", request.planner_id.c_str());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return res;
    }

    // Get start and goal pos.
    // TODO: somehow handle pose targets, not just config targets.
    robot_state::RobotState state = *robot_->getScratchState();  // make a copy of the states
    robot_state::robotStateMsgToRobotState(request.start_state, state);

    std::vector<double> start_joints;
    state.copyJointGroupPositions(request.group_name, start_joints);

    // TODO: extend Tesseract Planning to handle goal sampling?
    //     : already extened to handle arbitrary OMPL goals.
    std::vector<double> goal_joints;
    for (std::size_t i = 0; i < request.goal_constraints.size(); i++)
    {
        // We will only use the first constraint that has joints.
        if (request.goal_constraints[i].joint_constraints.empty())
            continue;

        std::map<std::string, double> positions;
        for (const auto &joint_cnt : request.goal_constraints[i].joint_constraints)
            positions[joint_cnt.joint_name] = joint_cnt.position;

        state.setVariablePositions(positions);
        state.update();

        state.copyJointGroupPositions(request.group_name, goal_joints);
    }

    if (goal_joints.empty())
    {
        ROS_ERROR("We can only handle Joint Goals at the moment. Sorry.");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return res;
    }

    tesseract::tesseract_planning::OmplPlanParameters params;
    params.planning_time = request.allowed_planning_time;
    params.simplify = settings_.simplify_solutions;

    // Call plan.
    ompl::time::point start = ompl::time::now();
    auto maybe_path = chain_interface_->plan(planner, start_joints, goal_joints, params);
    ompl::time::duration total_time = ompl::time::now() - start;

    if (maybe_path)
    {
        const ompl::geometric::PathGeometric &path = *maybe_path;
        res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robot_->getModel(), request.group_name));
        for (std::size_t i = 0; i < path.getStateCount(); i++)
        {
            state.setJointGroupPositions(
                request.group_name,
                path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values);
            state.update();
            res.trajectory_->addSuffixWayPoint(state, 0.0);
        }

        res.planning_time_ = ompl::time::seconds(total_time);
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return res;
    }
    else
    {
        ROS_WARN("Planning failed");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return res;
    }
}

const std::vector<std::string> OMPLChainPlanner::getPlannerConfigs() const
{
    std::vector<std::string> toReturn;
    toReturn.reserve(known_planners_.size());

    for (const auto &known_planner : known_planners_)
        toReturn.emplace_back(known_planner.first);

    return toReturn;
}
