/* Author: Bryce Willey */

#include <tesseract_planners.h>

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

using namespace robowflex::robow_tesseract;


namespace
{
    using namespace robowflex::robow_tesseract;
    
    template <typename T>
    static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr& si, const std::string &new_name, std::map<std::string, std::string> config)
    {
        ompl::base::PlannerPtr planner(new T(si));
        if (not new_name.empty())
        {
            planner->setName(new_name);
        }
        planner->params().setParams(config, true);
        planner->setup();
        return planner;
    }
}


OMPLChainPlanner::Settings()
  : simplify_solutions(true)
  , max_solution_segment_length(0.0)
  , use_continuous_validator(false)
{
}

void OMPLChainPlanner::registerDefaultPlanners()
{
    known_planners_["geometric::RRT"] = std::bind(&allocatePlanner<ompl::geometric::RRT>, _1, _2, _3));
    known_planners_["geometric::RRTConnect"] = std::bind(&allocatePlanner<ompl::geometric::RRTConnect>, _1, _2, _3));
    known_planners_["geometric::LazyRRT"] = std::bind(&allocatePlanner<ompl::geometric::LazyRRT>, _1, _2, _3));
    known_planners_["geometric::RRTstar"] = std::bind(&allocatePlanner<ompl::geometric::RRTstar>, _1, _2, _3));
    known_planners_["geometric::InformedRRTstar"] = std::bind(&allocatePlanner<ompl::geometric::InformedRRTstar>, _1, _2, _3));
    known_planners_["geometric::TRRT"] = std::bind(&allocatePlanner<ompl::geometric::TRRT>, _1, _2, _3));
    known_planners_["geometric::BiTRRT"] = std::bind(&allocatePlanner<ompl::geometric::BiTRRT>, _1, _2, _3));
    known_planners_["geometric::LBTRRT"] = std::bind(&allocatePlanner<ompl::geometric::LBTRRT>, _1, _2, _3));
    known_planners_["geometric::LazyLBTRRT"] = std::bind(&allocatePlanner<ompl::geometric::LazyLBTRRT>, _1, _2, _3));
    known_planners_["geometric::EST"] = std::bind(&allocatePlanner<ompl::geometric::EST>, _1, _2, _3));
    known_planners_["geometric::ProjEST"] = std::bind(&allocatePlanner<ompl::geometric::ProjEST>, _1, _2, _3));
    known_planners_["geometric::SBL"] = std::bind(&allocatePlanner<ompl::geometric::SBL>, _1, _2, _3));
    known_planners_["geometric::KPIECE"] = std::bind(&allocatePlanner<ompl::geometric::KPIECE>, _1, _2, _3));
    known_planners_["geometric::BKPIECE"] = std::bind(&allocatePlanner<ompl::geometric::BKPIECE>, _1, _2, _3));
    known_planners_["geometric::LBKPIECE"] = std::bind(&allocatePlanner<ompl::geometric::LBKPIECE>, _1, _2, _3));
    known_planners_["geometric::PRM"] = std::bind(&allocatePlanner<ompl::geometric::PRM>, _1, _2, _3));
    known_planners_["geometric::PRMstar"] = std::bind(&allocatePlanner<ompl::geometric::PRMstar>, _1, _2, _3));
    known_planners_["geometric::LazyPRM"] = std::bind(&allocatePlanner<ompl::geometric::LazyPRM>, _1, _2, _3));
    known_planners_["geometric::LazyPRMstar"] = std::bind(&allocatePlanner<ompl::geometric::LazyPRMstar>, _1, _2, _3));
    known_planners_["geometric::SPARS"] = std::bind(&allocatePlanner<ompl::geometric::SPARS>, _1, _2, _3));
    known_planners_["geometric::SPARStwo"] = std::bind(&allocatePlanner<ompl::geometric::SPARStwo>, _1, _2, _3));
    known_planners_["geometric::FMT"] = std::bind(&allocatePlanner<ompl::geometric::FMT>, _1, _2, _3));
    known_planners_["geometric::BFMT"] = std::bind(&allocatePlanner<ompl::geometric::BFMT>, _1, _2, _3));
    known_planners_["geometric::BITstar"] = std::bind(&allocatePlanner<ompl::geometric::BITstar>, _1, _2, _3));
    known_planners_["geometric::PDST"] = std::bind(&allocatePlanner<ompl::geometric::PDST>, _1, _2, _3));
    known_planners_["geometric::STRIDE"] = std::bind(&allocatePlanner<ompl::geometric::STRIDE>, _1, _2, _3));
}

bool OMPLChainPlanner::initialize(const std::string &config_file, const OMPLChainPlanner::Settings &settings)
{
    // TODO: Actually do something with the config file, for now it seems like we have to duplicate all of MoveIt's ompl_interface.
    //if (not OMPL::loadOMPLConfig(handler_, config_file, configs_))
    //    return false;

    registerDefaultPlanners();

    settings_.simplify_solutions = settings.simplify_solutions;
    settings_.max_solution_segment_length = settings.max_solution_segment_length;
    settings_.use_continuous_validator = settings.use_continuous_validator;
}

planning_interface::MotionPlanResponse OMPLChainPlanner::plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override
{
    planning_interface::MotionPlanResponse res;

    tesseract::tesseract_ros::KDLEnvPtr env = constructTesseractEnv(scene, getRobot());

    // Make the OMPL Context.
    chain_interface_ = std::make_shared<tesseract::tesseract_planning::ChainOmplInterface>(env, request.group_name);

    // Set the Motion Validator in chain_interface_.
    if (settings_.use_continuous_validator)
    {
        ompl::base::MotionValidatorPtr mv(new tesseract::tesseract_planning::ContinuousMotionValidator(chain_interface->spaceInformation(), env, request.group_name));
        chain_interface->setMotionValidator(mv);
    }

    ompl::base::PlannerPtr planner;
    auto it = known_planners_.find(request.planner_id);
    if (it != known_planners_.end())
    {
        std::map<std::string, std::string> config; // TODO: get the actual version of this.
        planner = it->second(chain_interface_->spaceInformation(), "tesseract_planner?", config);
    }
    else
    {
        ROS_ERROR("Unknown planner: '%s'", request.planner_id.c_str());
        res.error_code = moveit_msgs::MoveItErrorCodes::FAILURE;
        return res;
    }

    // Get start and goal pos.
    // TODO: somehow handle pose targets, not just config targets.
    robot_state::RobotStatePtr state; // TODO: get this somehow.
    moveit::core::robotStateMsgToRobotState(request_.start_state, *state);
    std::vector<double> start_joints; 
    state->copyJointGroupPositions(request.group_name, start_joints);

    // TODO: extend Tesseract Planning to handle goal sampling?
    //     : already extened to handle arbitrary OMPL goals.
    std::vector<double> goal_joints;
    for (std::size_t i = 0; i < request.goal_constraints.size(); i++)
    {
        // We will only use the first constraint that has joints.
        if (request.goal_constraints[i].joint_constraints.size() == 0)
        {
            continue;
        }
        std::map<std::string, double> positions;
        for (auto joint_cnt : request.goal_constraints[i].joint_constraints)
        {
            positions[joint_cnt.joint_name] = joint_cnt.position;
        }
        state->setVariablePositions(positions);
        state->update();
        state->copyJointGroupPositions(request.group_name, goal_joints);
    }
    if (goal_joints.empty())
    {
        ROS_ERROR("We can only handle Joint Goals at the moment. Sorry.");
        res.error_code = moveit_msgs::MoveItErrorCodes::FAILURE;
        return res;
    }

    tesseract::tesseract_planning::OmplPlanParameters params;
    params.planning_time = request.planning_time;
    params.simplify = settings_.simplify_solutions;

    // Call plan.
    auto maybe_path = chain_interface_->plan(planner, start_joints, goal_joints, params);
    if (maybe_path)
    {
        const ompl::geometric::PathGeometric& path = *maybe_path;
    }
    else
    {
        ROS_WARN("Planning failed");
    }


}