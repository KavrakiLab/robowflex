/* Author: Carlos Quintero */

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// Robowflex
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

// Tesseract
#include <tesseract_msgs/AttachableObject.h>
#include <trajopt/file_write_callback.hpp>

// TrajOptPlanner
#include <robowflex_tesseract/conversions.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

TrajOptPlanner::TrajOptPlanner(const RobotPtr &robot, const std::string &group_name, const std::string &manip)
  : Planner(robot, "trajopt"), group_(group_name), manip_(manip)
{
}

bool TrajOptPlanner::initialize()
{
    // Start KDL environment with the robot information.
    env_ = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
    if (!env_->init(robot_->getURDF(), robot_->getSRDF()))
    {
        ROS_ERROR("Error loading robot %s", robot_->getName());
        return false;
    }

    // Check if manipulator was correctly loaded.
    if (!env_->hasManipulator(manip_))
    {
        ROS_ERROR("No manipulator found in KDL environment");
        return false;
    }

    // Initialize trajectory.
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);

    return true;
}

void TrajOptPlanner::setInitialTrajectory(const TrajArray &init_trajectory)
{
    initial_trajectory_ = init_trajectory;
    init_type_ = InitInfo::Type::GIVEN_TRAJ;
}

void TrajOptPlanner::setInitType(const InitInfo::Type &init_type)
{
    if (init_type != InitInfo::Type::GIVEN_TRAJ)
        init_type_ = init_type;
    else
        ROS_ERROR("init type can only be set to GIVEN_TRAJ calling the giveInitialTrajectory() function");
}

const robot_trajectory::RobotTrajectoryPtr &TrajOptPlanner::getTrajectory() const
{
    return trajectory_;
}

const std::vector<std::string> &TrajOptPlanner::getEnvironmentLinks() const
{
    return env_->getLinkNames();
}

const std::vector<std::string> &TrajOptPlanner::getManipulatorLinks() const
{
    return env_->getManipulator(manip_)->getLinkNames();
}

const std::vector<std::string> &TrajOptPlanner::getManipulatorJoints() const
{
    return env_->getManipulator(manip_)->getJointNames();
}

planning_interface::MotionPlanResponse TrajOptPlanner::plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse res;

    // Extract start state.
    auto start_state = robot_->allocState();
    moveit::core::robotStateMsgToRobotState(request.start_state, *start_state);
    start_state->update(true);
    
    // Extract goal state.
    auto goal_state = robot_->allocState();
    if (request.goal_constraints.size() != 1)
    {
        ROS_ERROR("Ambiguous goal, %lu goal goal_constraints exist, returning default goal",
                  request.goal_constraints.size());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return res;
    }
    if (request.goal_constraints[0].joint_constraints.empty())
    {
        ROS_ERROR("No joint constraints specified, returning default goal");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return res;
    }

    std::map<std::string, double> variable_map;
    for (const auto &joint : request.goal_constraints[0].joint_constraints)
        variable_map[joint.joint_name] = joint.position;

    // Start state includes attached objects and values for the non-group links.
    moveit::core::robotStateMsgToRobotState(request.start_state, *goal_state);
    goal_state->setVariablePositions(variable_map);
    goal_state->update(true);

    // Plan.
    if (plan(scene, start_state, goal_state))
    {
        res.trajectory_ = trajectory_;
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else
    {
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }

    return res;
}

bool TrajOptPlanner::plan(const SceneConstPtr &scene, const robot_state::RobotStatePtr &start_state,
                          const robot_state::RobotStatePtr &goal_state)
{
    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add start state.
        addStartState(start_state, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal state.
        addGoalState(goal_state, pci);

        return solve(pci);
    }

    return false;
}

bool TrajOptPlanner::plan(const SceneConstPtr &scene, const robot_state::RobotStatePtr &start_state, const Eigen::Isometry3d &goal_pose, const std::string &link)
{
    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
    {
        ROS_ERROR("Straight line interpolation can not be done with a goal_pose.");
        return false;
    }

    auto begin_it = env_->getManipulator(manip_)->getLinkNames().begin();
    auto end_it = env_->getManipulator(manip_)->getLinkNames().end();
    auto link_it = std::find(begin_it, end_it, link);
    if (link_it == end_it)
    {
        ROS_ERROR("Link %s is not part of robot manipulator", link.c_str());
        return false;
    }

    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add start state
        addStartState(start_state, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal pose for link.
        addGoalPose(goal_pose, link, pci);

        return solve(pci);
    }

    return false;
}

bool TrajOptPlanner::plan(const SceneConstPtr &scene,
                          const std::unordered_map<std::string, double> &start_state,
                          const Eigen::Isometry3d &goal_pose, const std::string &link)
{
    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
    {
        ROS_ERROR("Straight line interpolation can not be done with a goal_pose.");
        return false;
    }

    auto begin_it = env_->getManipulator(manip_)->getLinkNames().begin();
    auto end_it = env_->getManipulator(manip_)->getLinkNames().end();
    auto link_it = std::find(begin_it, end_it, link);
    if (link_it == end_it)
    {
        ROS_ERROR("Link %s is not part of robot manipulator", link.c_str());
        return false;
    }

    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add start state
        addStartState(start_state, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal pose for link.
        addGoalPose(goal_pose, link, pci);

        return solve(pci);
    }

    return false;
}

bool TrajOptPlanner::plan(const SceneConstPtr &scene, const Eigen::Isometry3d &start_pose,
                          const std::string &start_link, const Eigen::Isometry3d &goal_pose,
                          const std::string &goal_link)
{
    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
    {
        ROS_ERROR("Straight line interpolation can not be done with a start_pose or a goal_pose");
        return false;
    }

    auto begin_it = env_->getManipulator(manip_)->getLinkNames().begin();
    auto end_it = env_->getManipulator(manip_)->getLinkNames().end();
    auto start_it = std::find(begin_it, end_it, start_link);
    auto goal_it = std::find(begin_it, end_it, goal_link);
    if ((start_it == end_it) or (goal_it == end_it))
    {
        ROS_ERROR("Given links %s or %s are not part of robot manipulator", start_link.c_str(),
                  goal_link.c_str());
        return false;
    }

    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Fill in the problem construction info and initialization
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add start_pose for start_link.
        addStartPose(start_pose, start_link, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal_pose for goal_link.
        addGoalPose(goal_pose, goal_link, pci);

        return solve(pci);
    }

    return false;
}

std::vector<std::string> TrajOptPlanner::getPlannerConfigs() const
{
    std::vector<std::string> config;
    config.push_back("trajopt");
    return config;
}

void TrajOptPlanner::setWriteFile(bool file_write_cb, const std::string &file_path)
{
    file_write_cb_ = file_write_cb;
    stream_ptr_ = std::make_shared<std::ofstream>();
    boost::filesystem::path path(file_path);
    path /= "file_output.csv";
    file_path_ = path.string();
    stream_ptr_->open(file_path_, std::ofstream::out | std::ofstream::trunc);
}

void TrajOptPlanner::problemConstructionInfo(std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    pci->kin = env_->getManipulator(manip_);
    pci->basic_info.n_steps = options.num_waypoints;
    pci->basic_info.manip = manip_;
    pci->basic_info.dt_lower_lim = options.dt_lower_lim;
    pci->basic_info.dt_upper_lim = options.dt_upper_lim;
    pci->basic_info.start_fixed = options.start_fixed;
    pci->basic_info.use_time = options.use_time;
    pci->init_info.type = init_type_;
    pci->init_info.dt = options.init_info_dt;
    if (init_type_ == InitInfo::Type::GIVEN_TRAJ)
        pci->init_info.data = initial_trajectory_;

    ROS_INFO("TrajOpt initialization: %d", init_type_);

    // Add joint velocity cost (without time) to penalize longer paths.
    auto jv = std::make_shared<JointVelTermInfo>();
    jv->targets = std::vector<double>(pci->kin->numJoints(), 0.0);
    jv->coeffs = std::vector<double>(pci->kin->numJoints(), options.joint_vel_coeffs);
    jv->term_type = TT_COST;
    jv->first_step = 0;
    jv->last_step = options.num_waypoints - 1;
    jv->name = "joint_velocity_cost";
    pci->cost_infos.push_back(jv);
}

void TrajOptPlanner::addCollisionAvoidance(std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision_cost";
    collision->term_type = TT_COST;
    collision->continuous = cont_cc_;
    collision->first_step = 0;
    collision->last_step = options.num_waypoints - 1;
    collision->gap = options.collision_gap;
    collision->info = createSafetyMarginDataVector(
        pci->basic_info.n_steps, options.default_safety_margin, options.default_safety_margin_coeffs);
    pci->cost_infos.push_back(collision);
}

void TrajOptPlanner::addStartState(const MotionRequestBuilderPtr &request,
                                   std::shared_ptr<ProblemConstructionInfo> &pci)
{
    // Extract start state from request.
    addStartState(request->getStartConfiguration(), pci);
}

void TrajOptPlanner::addStartState(const robot_state::RobotStatePtr &start_state,
                                   std::shared_ptr<ProblemConstructionInfo> &pci)
{
    // Extract start state values.
    std::vector<double> values(start_state->getVariablePositions(),
                               start_state->getVariablePositions() + (int)start_state->getVariableCount());

    // Set the start_state into the Tesseract environment.
    env_->setState(start_state->getVariableNames(), values);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartState(const std::unordered_map<std::string, double> &start_state,
                                   std::shared_ptr<ProblemConstructionInfo> &pci)
{
    // Set the start_state into the Tesseract environment.
    env_->setState(start_state);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartPose(const Eigen::Isometry3d &start_pose, const std::string &link,
                                  std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    Eigen::Quaterniond rotation(start_pose.linear());
    auto pose_constraint = std::make_shared<CartPoseTermInfo>();
    pose_constraint->term_type = TT_CNT;
    pose_constraint->link = link;
    pose_constraint->timestep = 0;
    pose_constraint->xyz = start_pose.translation();
    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d::Constant(options.pose_cnt_pos_coeffs);
    pose_constraint->rot_coeffs = Eigen::Vector3d::Constant(options.pose_cnt_rot_coeffs);
    pose_constraint->name = "start_pose_cnt_link_" + link;
    pci->cnt_infos.push_back(pose_constraint);
}

void TrajOptPlanner::addGoalState(const MotionRequestBuilderPtr &request,
                                  std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    // Extract goal_state from request.
    addGoalState(request->getGoalConfiguration(), pci);
}

void TrajOptPlanner::addGoalState(const robot_state::RobotStatePtr &goal_state,
                                  std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    const auto &manip_joint_names = env_->getManipulator(manip_)->getJointNames();

    // Transform (robot) goal_state to manip state.
    std::vector<double> goal_state_values;
    hypercube::robotStateToManipState(goal_state, manip_joint_names, goal_state_values);

    // Add goal state as a joint constraint.
    addGoalState(goal_state_values, pci);
}

void TrajOptPlanner::addGoalState(const std::vector<double> goal_state,
                                  std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    auto joint_pos_constraint = std::make_shared<JointPosTermInfo>();
    joint_pos_constraint->term_type = TT_CNT;
    joint_pos_constraint->name = "goal_state_cnt";
    joint_pos_constraint->coeffs = std::vector<double>(pci->kin->numJoints(), options.joint_pos_cnt_coeffs);
    joint_pos_constraint->targets = goal_state;
    joint_pos_constraint->first_step = options.num_waypoints - 1;
    joint_pos_constraint->last_step = options.num_waypoints - 1;
    pci->cnt_infos.push_back(joint_pos_constraint);

    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
        pci->init_info.data =
            Eigen::Map<const Eigen::VectorXd>(goal_state.data(), static_cast<long int>(goal_state.size()));
}

void TrajOptPlanner::addGoalPose(const RobotPose &goal_pose, const std::string &link,
                                 std::shared_ptr<ProblemConstructionInfo> &pci) const
{
    Eigen::Quaterniond rotation(goal_pose.linear());
    auto pose_constraint = std::make_shared<CartPoseTermInfo>();
    pose_constraint->term_type = TT_CNT;
    pose_constraint->link = link;
    pose_constraint->timestep = options.num_waypoints - 1;
    pose_constraint->xyz = goal_pose.translation();
    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d::Constant(options.pose_cnt_pos_coeffs);
    pose_constraint->rot_coeffs = Eigen::Vector3d::Constant(options.pose_cnt_rot_coeffs);
    pose_constraint->name = "goal_pose_cnt_link_" + link;
    pci->cnt_infos.push_back(pose_constraint);
}

bool TrajOptPlanner::solve(const std::shared_ptr<ProblemConstructionInfo> &pci)
{
    // TrajOpt problem and optimizer parameters.
    TrajOptProbPtr prob = ConstructProblem(*pci);
    auto config = std::make_shared<tesseract::tesseract_planning::TrajOptPlannerConfig>(prob);
    setOptimizerParameters(config);
    
    if (file_write_cb_)
    {
        if (!stream_ptr_->is_open())
            stream_ptr_->open(file_path_, std::ofstream::out | std::ofstream::trunc);
        config->callbacks.push_back(WriteCallback(stream_ptr_, prob));
    }

    // Create the Tesseract Planner and response.
    tesseract::tesseract_planning::TrajOptPlanner planner;
    tesseract::tesseract_planning::PlannerResponse response;

    // Solve planning problem.
    bool success_plan = planner.solve(response, *config);
    if (success_plan)
    {
        // Clear previous trajectory and update it with new one.
        trajectory_->clear();
        updateTrajFromTesseractRes(response);

        std::cout << response.trajectory << std::endl;
        std::cout << response.status_description << std::endl;
    }

    // Write optimization results in file.
    if (file_write_cb_)
        stream_ptr_->close();

    return success_plan;
}

void TrajOptPlanner::updateTrajFromTesseractRes(
    const tesseract::tesseract_planning::PlannerResponse &response)
{
    for (int i = 0; i < response.trajectory.rows(); i++)
    {
        // Create a tmp state for every waypoint.
        auto tmp_state = robot_->allocState();

        // Transform tesseract manip ith waypoint to robot state.
        hypercube::manipStateToRobotState(response.trajectory.row(i), manip_, env_, tmp_state);

        // Add waypoint to trajectory.
        trajectory_->addSuffixWayPoint(tmp_state, 0.0);
    }
}

void TrajOptPlanner::setOptimizerParameters(std::shared_ptr<tesseract::tesseract_planning::TrajOptPlannerConfig> &config) const
{
    config->params.improve_ratio_threshold = options.improve_ratio_threshold;
    config->params.min_trust_box_size = options.min_trust_box_size;
    config->params.min_approx_improve = options.min_approx_improve;
    config->params.min_approx_improve_frac = options.min_approx_improve_frac;
    config->params.max_iter = options.max_iter;
    config->params.trust_shrink_ratio = options.trust_shrink_ratio;
    config->params.trust_expand_ratio = options.trust_expand_ratio;
    config->params.cnt_tolerance = options.cnt_tolerance;
    config->params.max_merit_coeff_increases = options.max_merit_coeff_increases;
    config->params.merit_coeff_increase_ratio = options.merit_coeff_increase_ratio;
    config->params.max_time = options.max_time;
    config->params.merit_error_coeff = options.merit_error_coeff;
    config->params.trust_box_size = options.trust_box_size;
}
