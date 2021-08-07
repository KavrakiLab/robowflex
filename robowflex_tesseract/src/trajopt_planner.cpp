/* Author: Carlos Quintero Pena */

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// Robowflex
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/util.h>
#include <robowflex_library/trajectory.h>

// Tesseract
#include <trajopt/file_write_callback.hpp>

// TrajOptPlanner
#include <robowflex_tesseract/conversions.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;
using namespace trajopt;

TrajOptPlanner::TrajOptPlanner(const RobotPtr &robot, const std::string &group_name, const std::string &name)
  : Planner(robot, name), group_(group_name)
{
}

bool TrajOptPlanner::initialize(const std::string &manip)
{
    // Save manipulator name.
    manip_ = manip;

    // Start KDL environment with the robot information.
    env_ = std::make_shared<tesseract::tesseract_ros::KDLEnv>();

    if (!env_->init(robot_->getURDF(), robot_->getSRDF()))
    {
        RBX_ERROR("Error loading robot %s", robot_->getName());
        return false;
    }

    // Check if manipulator was correctly loaded.
    if (!env_->hasManipulator(manip_))
    {
        RBX_ERROR("No manipulator found in KDL environment");
        return false;
    }

    // Initialize trajectory.
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);

    return true;
}

bool TrajOptPlanner::initialize(const std::string &base_link, const std::string &tip_link)
{
    // Save manipulator name.
    manip_ = "manipulator";

    // Start KDL environment with the robot information.
    env_ = std::make_shared<tesseract::tesseract_ros::KDLEnv>();

    if (!robot_->getModelConst()->hasLinkModel(base_link))
    {
        RBX_ERROR("%s does not exist in robot description", base_link);
        return false;
    }

    if (!robot_->getModelConst()->hasLinkModel(tip_link))
    {
        RBX_ERROR("%s does not exist in robot description", tip_link);
        return false;
    }

    if (options.verbose)
        RBX_INFO("Adding manipulator %s from %s to %s", manip_, base_link, tip_link);

    TiXmlDocument srdf_doc;
    srdf_doc.Parse(robot_->getSRDFString().c_str());

    auto *group_element = new TiXmlElement("group");
    group_element->SetAttribute("name", manip_.c_str());
    srdf_doc.FirstChildElement("robot")->LinkEndChild(group_element);

    auto *chain_element = new TiXmlElement("chain");
    chain_element->SetAttribute("base_link", base_link.c_str());
    chain_element->SetAttribute("tip_link", tip_link.c_str());
    group_element->LinkEndChild(chain_element);

    srdf::ModelSharedPtr srdf;
    srdf.reset(new srdf::Model());
    srdf->initXml(*(robot_->getURDF()), &srdf_doc);

    if (!env_->init(robot_->getURDF(), srdf))
    {
        RBX_ERROR("Error loading robot %s", robot_->getName());
        return false;
    }

    // Check if manipulator was correctly loaded.
    if (!env_->hasManipulator(manip_))
    {
        RBX_ERROR("No manipulator found in KDL environment");
        return false;
    }

    // Initialize trajectory.
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);

    return true;
}

void TrajOptPlanner::setInitialTrajectory(const robot_trajectory::RobotTrajectoryPtr &init_trajectory)
{
    hypercube::robotTrajToManipTesseractTraj(init_trajectory, manip_, env_, initial_trajectory_);
    init_type_ = InitInfo::Type::GIVEN_TRAJ;
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
        RBX_ERROR("init type can only be set to GIVEN_TRAJ calling the setInitialTrajectory() function");
}

const robot_trajectory::RobotTrajectoryPtr &TrajOptPlanner::getTrajectory() const
{
    return trajectory_;
}

const trajopt::TrajArray &TrajOptPlanner::getTesseractTrajectory() const
{
    return tesseract_trajectory_;
}

const std::vector<std::string> &TrajOptPlanner::getEnvironmentLinks() const
{
    if (env_->checkInitialized())
        return env_->getLinkNames();
    else
        throw Exception(1, "KDL environment not initialized with robot links!");
}

const std::vector<std::string> &TrajOptPlanner::getManipulatorLinks() const
{
    if (env_->hasManipulator(manip_))
        return env_->getManipulator(manip_)->getLinkNames();
    else
        throw Exception(1, "There is no loaded manipulator!");
}

const std::vector<std::string> &TrajOptPlanner::getManipulatorJoints() const
{
    if (env_->hasManipulator(manip_))
        return env_->getManipulator(manip_)->getJointNames();
    else
        throw Exception(1, "There is no loaded manipulator!");
}

double TrajOptPlanner::getPlanningTime() const
{
    return time_;
}

void TrajOptPlanner::fixJoints(const std::vector<std::string> &joints)
{
    if (!env_->hasManipulator(manip_))
        throw Exception(1, "There is no loaded manipulator!");
    else
    {
        const auto &joint_names = env_->getManipulator(manip_)->getJointNames();
        for (const auto &name : joints)
        {
            auto it = std::find(joint_names.begin(), joint_names.end(), name);
            if (it == joint_names.end())
                throw Exception(1, "One of the joints to be fixed does not exist");
            else
            {
                int index = std::distance(joint_names.begin(), it);
                fixed_joints_.push_back(index);
            }
        }
    }
}

planning_interface::MotionPlanResponse
TrajOptPlanner::plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse res;

    // Extract start state.
    auto start_state = robot_->allocState();
    moveit::core::robotStateMsgToRobotState(request.start_state, *start_state);
    start_state->update(true);

    // Use the start state as reference state to build trajectory_.
    ref_state_ = std::make_shared<robot_state::RobotState>(*start_state);

    // Extract goal state.
    auto goal_state = robot_->allocState();
    if (request.goal_constraints.size() != 1)
    {
        RBX_ERROR("Ambiguous goal, %lu goal goal_constraints exist, returning default goal",
                  request.goal_constraints.size());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return res;
    }
    if (request.goal_constraints[0].joint_constraints.empty())
    {
        RBX_ERROR("No joint constraints specified, returning default goal");
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

    // If planner runs until timeout, use the allowed_planning_time from the request.
    if (!options.return_first_sol)
        options.max_planning_time = request.allowed_planning_time;

    // Plan.
    auto result = plan(scene, start_state, goal_state);
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    if (result.first and result.second)
    {
        res.planning_time_ = time_;
        res.trajectory_ = trajectory_;
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }

    return res;
}

TrajOptPlanner::PlannerResult TrajOptPlanner::plan(const SceneConstPtr &scene,
                                                   const robot_state::RobotStatePtr &start_state,
                                                   const robot_state::RobotStatePtr &goal_state)
{
    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Attach bodies to KDL env.
        hypercube::addAttachedBodiesToTesseractEnv(ref_state_, env_);

        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add velocity cost.
        addVelocityCost(pci);

        // Add start state.
        addStartState(start_state, pci);

        // Add collision costs to all waypoints in the trajectory.
        options.default_safety_margin_coeffs = options.joint_state_safety_margin_coeffs;
        addCollisionAvoidance(pci);

        // Add goal state.
        addGoalState(goal_state, pci);

        return solve(scene, pci);
    }

    return PlannerResult(false, false);
}

TrajOptPlanner::PlannerResult TrajOptPlanner::plan(const SceneConstPtr &scene,
                                                   const robot_state::RobotStatePtr &start_state,
                                                   const RobotPose &goal_pose, const std::string &link)
{
    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
    {
        RBX_ERROR("Straight line interpolation can not be done with a goal_pose.");
        return PlannerResult(false, false);
    }

    auto begin_it = env_->getManipulator(manip_)->getLinkNames().begin();
    auto end_it = env_->getManipulator(manip_)->getLinkNames().end();
    auto link_it = std::find(begin_it, end_it, link);
    if (link_it == end_it)
    {
        RBX_ERROR("Link %s is not part of robot manipulator", link);
        return PlannerResult(false, false);
    }

    // Use the start state as reference state to build trajectory_.
    ref_state_ = std::make_shared<robot_state::RobotState>(*start_state);

    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Attach bodies to KDL env.
        hypercube::addAttachedBodiesToTesseractEnv(ref_state_, env_);

        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add velocity cost.
        addVelocityCost(pci);

        // Add start state
        addStartState(start_state, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal pose for link.
        addGoalPose(goal_pose, link, pci);

        return solve(scene, pci);
    }

    return PlannerResult(false, false);
}

TrajOptPlanner::PlannerResult TrajOptPlanner::plan(const SceneConstPtr &scene,
                                                   const std::unordered_map<std::string, double> &start_state,
                                                   const RobotPose &goal_pose, const std::string &link)
{
    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
    {
        RBX_ERROR("Straight line interpolation can not be done with a goal_pose.");
        return PlannerResult(false, false);
    }

    auto begin_it = env_->getManipulator(manip_)->getLinkNames().begin();
    auto end_it = env_->getManipulator(manip_)->getLinkNames().end();
    auto link_it = std::find(begin_it, end_it, link);
    if (link_it == end_it)
    {
        RBX_ERROR("Link %s is not part of robot manipulator", link);
        return PlannerResult(false, false);
    }

    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add velocity cost.
        addVelocityCost(pci);

        // Add start state
        addStartState(start_state, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal pose for link.
        addGoalPose(goal_pose, link, pci);

        return solve(scene, pci);
    }

    return PlannerResult(false, false);
}

TrajOptPlanner::PlannerResult TrajOptPlanner::plan(const SceneConstPtr &scene, const RobotPose &start_pose,
                                                   const std::string &start_link, const RobotPose &goal_pose,
                                                   const std::string &goal_link)
{
    if (init_type_ == InitInfo::Type::JOINT_INTERPOLATED)
    {
        RBX_ERROR("Straight line interpolation can not be done with a start_pose or a goal_pose");
        return PlannerResult(false, false);
    }

    auto begin_it = env_->getManipulator(manip_)->getLinkNames().begin();
    auto end_it = env_->getManipulator(manip_)->getLinkNames().end();
    auto start_it = std::find(begin_it, end_it, start_link);
    auto goal_it = std::find(begin_it, end_it, goal_link);
    if ((start_it == end_it) or (goal_it == end_it))
    {
        RBX_ERROR("Given links %s or %s are not part of robot manipulator", start_link, goal_link);
        return PlannerResult(false, false);
    }

    // Create the tesseract environment from the scene.
    if (hypercube::sceneToTesseractEnv(scene, env_))
    {
        // Fill in the problem construction info and initialization
        auto pci = std::make_shared<ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add velocity cost.
        addVelocityCost(pci);

        // Add start_pose for start_link.
        addStartPose(start_pose, start_link, pci);

        // Add collision costs to all waypoints in the trajectory.
        addCollisionAvoidance(pci);

        // Add goal_pose for goal_link.
        addGoalPose(goal_pose, goal_link, pci);

        return solve(scene, pci);
    }

    return PlannerResult(false, false);
}

TrajOptPlanner::PlannerResult TrajOptPlanner::plan(const SceneConstPtr &scene,
                                                   const robot_state::RobotStatePtr &start_state)
{
    throw Exception(1, "You need to implement virtual method TrajOptPlanner::plan() in your derived class");
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
    if (file_write_cb_)
        stream_ptr_->open(file_path_, std::ofstream::out | std::ofstream::trunc);
}

void TrajOptPlanner::problemConstructionInfo(std::shared_ptr<ProblemConstructionInfo> pci) const
{
    pci->basic_info.convex_solver = options.backend_optimizer;
    pci->kin = env_->getManipulator(manip_);
    pci->basic_info.n_steps = options.num_waypoints;
    pci->basic_info.manip = manip_;
    pci->basic_info.dt_lower_lim = options.dt_lower_lim;
    pci->basic_info.dt_upper_lim = options.dt_upper_lim;
    pci->basic_info.start_fixed = options.start_fixed;
    pci->basic_info.dofs_fixed = fixed_joints_;
    pci->basic_info.use_time = options.use_time;
    pci->init_info.type = init_type_;
    pci->init_info.dt = options.init_info_dt;
    if (init_type_ == InitInfo::Type::GIVEN_TRAJ)
        pci->init_info.data = initial_trajectory_;

    if (options.verbose)
        RBX_INFO("TrajOpt initialization: %d", init_type_);
}

void TrajOptPlanner::addVelocityCost(std::shared_ptr<trajopt::ProblemConstructionInfo> pci) const
{
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

void TrajOptPlanner::addCollisionAvoidance(std::shared_ptr<ProblemConstructionInfo> pci) const
{
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision_cost";
    collision->term_type = TT_COST;
    collision->continuous = options.use_cont_col_avoid;
    collision->first_step = 0;
    collision->last_step = options.num_waypoints - 1;
    collision->gap = options.collision_gap;
    collision->info = createSafetyMarginDataVector(pci->basic_info.n_steps, options.default_safety_margin,
                                                   options.default_safety_margin_coeffs);
    pci->cost_infos.push_back(collision);
}

void TrajOptPlanner::addStartState(const MotionRequestBuilderPtr &request,
                                   std::shared_ptr<ProblemConstructionInfo> pci)
{
    // Extract start state from request.
    addStartState(request->getStartConfiguration(), pci);
}

void TrajOptPlanner::addStartState(const robot_state::RobotStatePtr &start_state,
                                   std::shared_ptr<ProblemConstructionInfo> pci)
{
    // Extract start state values.
    std::vector<double> values(start_state->getVariablePositions(),
                               start_state->getVariablePositions() + (int)start_state->getVariableCount());

    // Set the start_state into the Tesseract environment.
    env_->setState(start_state->getVariableNames(), values);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartState(const std::unordered_map<std::string, double> &start_state,
                                   std::shared_ptr<ProblemConstructionInfo> pci)
{
    // Set the start_state into the Tesseract environment.
    env_->setState(start_state);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartPose(const RobotPose &start_pose, const std::string &link,
                                  std::shared_ptr<ProblemConstructionInfo> pci) const
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
                                  std::shared_ptr<ProblemConstructionInfo> pci) const
{
    // Extract goal_state from request.
    addGoalState(request->getGoalConfiguration(), pci);
}

void TrajOptPlanner::addGoalState(const robot_state::RobotStatePtr &goal_state,
                                  std::shared_ptr<ProblemConstructionInfo> pci) const
{
    const auto &manip_joint_names = env_->getManipulator(manip_)->getJointNames();

    // Transform (robot) goal_state to manip state.
    std::vector<double> goal_state_values;
    hypercube::robotStateToManipState(goal_state, manip_joint_names, goal_state_values);

    // Add goal state as a joint constraint.
    addGoalState(goal_state_values, pci);
}

void TrajOptPlanner::addGoalState(const std::vector<double> goal_state,
                                  std::shared_ptr<ProblemConstructionInfo> pci) const
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
                                 std::shared_ptr<ProblemConstructionInfo> pci) const
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

TrajOptPlanner::PlannerResult TrajOptPlanner::solve(const SceneConstPtr &scene,
                                                    const std::shared_ptr<ProblemConstructionInfo> &pci)
{
    PlannerResult planner_result(false, false);

    // Create optimizer and populate parameters.
    TrajOptProbPtr prob = ConstructProblem(*pci);
    sco::BasicTrustRegionSQP opt(prob);
    opt.setParameters(getTrustRegionSQPParameters());

    // Add write file callback.
    if (file_write_cb_)
    {
        if (!stream_ptr_->is_open())
            stream_ptr_->open(file_path_, std::ofstream::out | std::ofstream::trunc);

        opt.addCallback(WriteCallback(stream_ptr_, prob));
    }

    // If the planner needs to run more than once, add noise to the initial trajectory.
    if (!options.return_first_sol)
        options.perturb_init_traj = true;

    // Initialize.
    double total_time = 0.0;
    double best_cost = std::numeric_limits<double>::infinity();

    while (true)
    {
        // Perturb initial trajectory if needed.
        auto init_trajectory = prob->GetInitTraj();
        if (options.perturb_init_traj)
        {
            // Perturb all waypoints but start and goal.
            int rows = options.num_waypoints - 2;
            int cols = pci->kin->numJoints();
            double noise = options.noise_init_traj;

            init_trajectory.block(1, 0, rows, cols) += (Eigen::MatrixXd::Constant(rows, cols, -noise) +
                                                        2 * noise *
                                                            (Eigen::MatrixXd::Random(rows, cols) * 0.5 +
                                                             Eigen::MatrixXd::Constant(rows, cols, 0.5)));
        }

        opt.initialize(trajToDblVec(init_trajectory));

        // Optimize.
        ros::Time tStart = ros::Time::now();
        opt.optimize();

        // Measure and print time.
        double time = (ros::Time::now() - tStart).toSec();
        total_time += time;
        time_ = total_time;

        if (opt.results().status == sco::OptStatus::OPT_CONVERGED)
        {
            // Optimization problem converged.
            planner_result.first = true;

            // Check for collisions.
            auto tss_current_traj = getTraj(opt.x(), prob->GetVars());
            auto current_traj =
                std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);
            hypercube::manipTesseractTrajToRobotTraj(tss_current_traj, ref_state_, manip_, env_,
                                                     current_traj);
            auto const &ct = std::make_shared<Trajectory>(current_traj);
            bool is_ct_collision_free = ct->isCollisionFree(scene);

            // If trajectory is better than current, update the trajectory and cost.
            if ((opt.results().total_cost < best_cost) and is_ct_collision_free)
            {
                // Update best cost.
                best_cost = opt.results().total_cost;

                // Clear current trajectory.
                trajectory_->clear();

                // Update trajectory.
                tesseract_trajectory_ = tss_current_traj;
                trajectory_ = current_traj;

                // Solution is collision-free.
                planner_result.second = true;
            }
        }

        if (options.return_first_sol or
            (options.return_after_timeout and (total_time >= options.max_planning_time)) or
            (!options.return_after_timeout and
             (planner_result.second or (total_time >= options.max_planning_time))))
            break;
    }

    // Print status
    if (options.verbose)
    {
        RBX_INFO("OPTIMIZATION STATUS: %s", sco::statusToString(opt.results().status));
        RBX_INFO("TOTAL PLANNING TIME: %.3f", time_);
        RBX_INFO("COST: %.3f", best_cost);
        RBX_INFO("COLLISION STATUS: %s", (planner_result.second) ? "COLLISION FREE" : "IN COLLISION");

        if (planner_result.first)
            RBX_INFO("\n%s", tesseract_trajectory_);
    }

    // Write optimization results in file.
    if (file_write_cb_)
        stream_ptr_->close();

    return planner_result;
}

sco::BasicTrustRegionSQPParameters TrajOptPlanner::getTrustRegionSQPParameters() const
{
    sco::BasicTrustRegionSQPParameters params;

    params.improve_ratio_threshold = options.improve_ratio_threshold;
    params.min_trust_box_size = options.min_trust_box_size;
    params.min_approx_improve = options.min_approx_improve;
    params.min_approx_improve_frac = options.min_approx_improve_frac;
    params.max_iter = options.max_iter;
    params.trust_shrink_ratio = options.trust_shrink_ratio;
    params.trust_expand_ratio = options.trust_expand_ratio;
    params.cnt_tolerance = options.cnt_tolerance;
    params.max_merit_coeff_increases = options.max_merit_coeff_increases;
    params.merit_coeff_increase_ratio = options.merit_coeff_increase_ratio;
    params.merit_error_coeff = options.merit_error_coeff;
    params.trust_box_size = options.trust_box_size;

    return params;
}
