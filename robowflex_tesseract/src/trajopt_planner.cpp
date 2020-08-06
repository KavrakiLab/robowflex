/* Author: Carlos Quintero */

// Robowflex
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

// Tesseract
#include <tesseract_msgs/AttachableObject.h>
#include <trajopt/plot_callback.hpp>

// TrajOptPlanner
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

TrajOptPlanner::TrajOptPlanner(const RobotConstPtr &robot, const std::string &group_name, const std::string &manip)
  : robot_(robot), group_(group_name), manip_(manip)
{
    env_ = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
    env_->init(robot_->getURDF(), robot_->getSRDF());
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);
}

void TrajOptPlanner::setInitType(const trajopt::InitInfo::Type &init_type)
{
    if (init_type != trajopt::InitInfo::Type::GIVEN_TRAJ) 
        init_type_ = init_type;
    else 
        ROS_ERROR("init type can only be set to GIVEN_TRAJ calling the giveInitialTrajectory() function");
}

bool TrajOptPlanner::plan(const SceneConstPtr &scene,
                          const MotionRequestBuilderPtr &request)
{
    return plan(scene, request->getStartConfiguration(), request->getGoalConfiguration());
}

bool TrajOptPlanner::plan(const SceneConstPtr &scene,
                          const robot_state::RobotStatePtr &start_state, 
                          const robot_state::RobotStatePtr &goal_state)
{
    // Create the tesseract environment from the scene.
    if (createTesseractEnvFromScene(scene))
    {
        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
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

bool TrajOptPlanner::plan(const SceneConstPtr &scene, 
                          const std::unordered_map<std::string, double> &start_state, 
                          const Eigen::Isometry3d &goal_pose, 
                          const std::string &link)
{
    //TODO: verify that link is part of the manipulator
    //TODO: make start_state a RobotState
    if (init_type_ == trajopt::InitInfo::Type::JOINT_INTERPOLATED)
    {
        ROS_ERROR("Straight line interpolation can not be done with a goal_pose");
        return false;
    }
    
    // Create the tesseract environment from the scene.
    if (createTesseractEnvFromScene(scene))
    {
        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
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
                          const Eigen::Isometry3d &start_pose, 
                          const std::string &start_link, 
                          const Eigen::Isometry3d &goal_pose, 
                          const std::string &goal_link)
{
    //TODO: verify that both links are part of the manipulator
    if (init_type_ == trajopt::InitInfo::Type::JOINT_INTERPOLATED)
    {
        ROS_ERROR("Straight line interpolation can not be done with a goal_pose");
        return false;
    }
    
    // Create the tesseract environment from the scene.
    if (createTesseractEnvFromScene(scene))
    {
        // Fill in the problem construction info and initialization
        auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
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

void TrajOptPlanner::setWriteFile(bool file_write_cb, const std::string &file_path)
{
    file_write_cb_ = file_write_cb;
    file_path_ = file_path + "/file_output.csv";
    stream_ptr_ = std::make_shared<std::ofstream>();
    stream_ptr_->open(file_path_, std::ofstream::out | std::ofstream::trunc);
}

void TrajOptPlanner::problemConstructionInfo(std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    pci->kin = env_->getManipulator(manip_);
    pci->basic_info.n_steps = num_waypoints_;//
    pci->basic_info.manip = manip_;
    pci->basic_info.dt_lower_lim = 2;    // 1/most time
    pci->basic_info.dt_upper_lim = 100;  // 1/least time
    pci->basic_info.start_fixed = true;
    pci->basic_info.use_time = false;
    pci->init_info.type = init_type_;
    pci->init_info.dt = 0.5;
    if (init_type_ == trajopt::InitInfo::Type::GIVEN_TRAJ)
        pci->init_info.data = initial_trajectory_;
    
    ROS_INFO("TrajOpt initialization: %d", init_type_);
    
    // Add joint velocity cost (without time) to penalize longer paths.
    auto jv = std::make_shared<trajopt::JointVelTermInfo>();
    jv->targets = std::vector<double>(pci->kin->numJoints(), 0.0);
    jv->coeffs = std::vector<double>(pci->kin->numJoints(), 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = num_waypoints_ - 1;
    jv->name = "joint_velocity_cost";
    pci->cost_infos.push_back(jv);
}

void TrajOptPlanner::addCollisionAvoidance(std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    //festd::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
    auto collision = std::make_shared<trajopt::CollisionTermInfo>();
    collision->name = "collision_cost";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = cont_cc_;
    collision->first_step = 0;
    collision->last_step = num_waypoints_ - 1;
    collision->gap = 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci->basic_info.n_steps, 0.025, 50);
    pci->cost_infos.push_back(collision);
}

void TrajOptPlanner::addStartState(const MotionRequestBuilderPtr &request, 
                                   std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Extract start state from request.
    addStartState(request->getStartConfiguration(), pci);
}

void TrajOptPlanner::addStartState(const robot_state::RobotStatePtr &start_state, 
                                   std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Extract start state values.
    double* state_values = new double((int)start_state->getVariableCount());
    state_values = start_state->getVariablePositions();
    std::vector<double> values(state_values, state_values+(int)start_state->getVariableCount());   
    
    // Set the start_state into the Tesseract environment
    env_->setState(start_state->getVariableNames(), values);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartState(const std::unordered_map<std::string, double> &start_state, 
                                   std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Set the start_state into the Tesseract environment.
    env_->setState(start_state);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartPose(const Eigen::Isometry3d &start_pose, const std::string &link, 
                                  std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    Eigen::Quaterniond rotation(start_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = link;
    pose_constraint->timestep = 0;
    pose_constraint->xyz = start_pose.translation();
    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "start_pose_cnt_link_" + link;
    pci->cnt_infos.push_back(pose_constraint);
}

void TrajOptPlanner::addGoalState(const MotionRequestBuilderPtr &request, 
                                  std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Extract goal_state from request.
    addGoalState(request->getGoalConfiguration(), pci);
}

void TrajOptPlanner::addGoalState(const robot_state::RobotStatePtr &goal_state, 
                                  std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Transform (robot) goal_state to manip state.
    std::vector<double> goal_state_values;
    roboStateToManipState(goal_state, goal_state_values);
    
    // Add goal state as a joint constraint.
    addGoalState(goal_state_values, pci);
}

void TrajOptPlanner::addGoalState(const std::vector<double> goal_state,
                                  std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    auto joint_pos_constraint = std::make_shared<trajopt::JointPosTermInfo>();
    joint_pos_constraint->term_type = trajopt::TT_CNT;
    joint_pos_constraint->name = "goal_state_cnt";
    joint_pos_constraint->coeffs = std::vector<double>(pci->kin->numJoints(), 5.0);
    joint_pos_constraint->targets = goal_state;
    joint_pos_constraint->first_step = num_waypoints_-1;
    joint_pos_constraint->last_step = num_waypoints_-1;
    pci->cnt_infos.push_back(joint_pos_constraint);
    
    if (init_type_ == trajopt::InitInfo::Type::JOINT_INTERPOLATED)
        pci->init_info.data = Eigen::Map<const Eigen::VectorXd>(goal_state.data(), static_cast<long int>(goal_state.size()));
}

void TrajOptPlanner::addGoalPose(const robowflex::RobotPose &goal_pose, const std::string &link, 
                                 std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    Eigen::Quaterniond rotation(goal_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = link;
    pose_constraint->timestep = num_waypoints_-1;
    pose_constraint->xyz = goal_pose.translation();
    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "goal_pose_cnt_link_" + link;
    pci->cnt_infos.push_back(pose_constraint);
}

bool TrajOptPlanner::solve(const std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // TrajOpt problem and optimizer parameters
    trajopt::TrajOptProbPtr prob = ConstructProblem(*pci);
    tesseract::tesseract_planning::TrajOptPlannerConfig config(prob);
    config.params.max_iter = 100;
    if (file_write_cb_)
        config.callbacks.push_back(trajopt::WriteCallback(stream_ptr_, prob));
    
    // Create the Tesseract Planner and response
    tesseract::tesseract_planning::TrajOptPlanner planner;
    tesseract::tesseract_planning::PlannerResponse response;
    
    // Solve planning problem
    if (planner.solve(response, config))
    {
        // Clean previous trajectory and update it with new one
        trajectory_->clear();
        updateTrajFromTesseractRes(response);

        // Write optimization results in file.
        if (file_write_cb_)
            stream_ptr_->close();
        
        std::cout << response.trajectory << std::endl;
        std::cout << response.status_description << std::endl;
        return true;
    }
    return false;
}

bool TrajOptPlanner::createTesseractEnvFromScene(const robowflex::SceneConstPtr &scene)
{
    if (env_->checkInitialized())
    {
        // Clear environment from scene objects.
        env_->clearAttachableObjects();
        env_->clearAttachedBodies();
        
        // Loop over collision objects in the scene.
        const auto &scene_msg = scene->getMessage();
        for (const auto &collision_object : scene_msg.world.collision_objects)
        {
            // Add collision and visual objects
            tesseract_msgs::AttachableObject obj;
            obj.name = collision_object.id;
            obj.operation = tesseract_msgs::AttachableObject::ADD;

            for (const shape_msgs::SolidPrimitive &primitive : collision_object.primitives)
            {
                obj.visual.primitives.emplace_back(primitive);
                obj.collision.primitives.emplace_back(primitive);
                std_msgs::Int32 cot;
                cot.data = tesseract::CollisionObjectType::UseShapeType;
                obj.collision.primitive_collision_object_types.emplace_back(cot);
            }

            for (const shape_msgs::Mesh &mesh : collision_object.meshes)
            {
                obj.visual.meshes.emplace_back(mesh);
                obj.collision.meshes.emplace_back(mesh);
            }

            for (const shape_msgs::Plane &plane : collision_object.planes)
            {
                obj.visual.planes.emplace_back(plane);
                obj.collision.planes.emplace_back(plane);
            }

            for (const geometry_msgs::Pose &pose : collision_object.primitive_poses)
            {
                obj.visual.primitive_poses.emplace_back(pose);
                obj.collision.primitive_poses.emplace_back(pose);
            }

            for (const geometry_msgs::Pose &pose : collision_object.mesh_poses)
            {
                obj.visual.mesh_poses.emplace_back(pose);
                obj.collision.mesh_poses.emplace_back(pose);
            }

            for (const geometry_msgs::Pose &pose : collision_object.plane_poses)
            {
                obj.visual.plane_poses.emplace_back(pose);
                obj.collision.plane_poses.emplace_back(pose);
            }

            // Add collision object as attachable object.
            auto ao = std::make_shared<tesseract::AttachableObject>();
            tesseract::tesseract_ros::attachableObjectMsgToAttachableObject(ao, obj);
            env_->addAttachableObject(ao);
            
            // Attach the object to the environment (to a parent frame).
            tesseract::AttachedBodyInfo attached_body_info;
            attached_body_info.object_name = collision_object.id;
            attached_body_info.parent_link_name = robot_->getModelConst()->getRootLinkName();
            attached_body_info.transform = Eigen::Isometry3d::Identity();
            env_->attachBody(attached_body_info);
        }
        ROS_INFO("Tesseract environment successfully created");
        return true;
    }
    ROS_ERROR("Tesseract environment not initialized");
    return false;
}

void TrajOptPlanner::updateTrajFromTesseractRes(const tesseract::tesseract_planning::PlannerResponse &response)
{
    for (int i=0;i<response.trajectory.rows();i++)
    {
        // Create a tmp state for every waypoint.
        auto tmp_state = robot_->allocState();
        
        // Transform tesseract manip ith waypoint to robot state.
        manipStateToRobotState(response.trajectory.row(i), tmp_state);
        
        // Add waypoint to trajectory.
        trajectory_->addSuffixWayPoint(tmp_state, 0.0);
    }
}

void TrajOptPlanner::roboStateToManipState(const robot_state::RobotStatePtr &robot_state, 
                                           std::vector<double> &manip_state)
{
    manip_state.clear();
    manip_state.resize(0);
    
    // Extract the state to a raw vector.
    double* raw_state_values = new double((int)robot_state->getVariableCount());
    raw_state_values = robot_state->getVariablePositions();
    
    // Loop over manip joints and add their values to goal_state in the correct order.
    const auto &robot_state_joint_names = robot_state->getVariableNames();
    for (const auto &joint_name : env_->getManipulator(manip_)->getJointNames())
    {            
        int index = std::distance(robot_state_joint_names.begin(),
                                  std::find(robot_state_joint_names.begin(), 
                                            robot_state_joint_names.end(), 
                                            joint_name)
                                 );
        manip_state.push_back(raw_state_values[index]);
    }
}

void TrajOptPlanner::manipStateToRobotState(const Eigen::Ref<const Eigen::VectorXd> &manip_state, 
                                            robot_state::RobotStatePtr &robot_state)
{
    // Initialize it with the env state (includes both group and non-group joints).
    const auto &joint_values = env_->getCurrentJointValues();
    std::vector<double> tmp_current_values(joint_values.data(), joint_values.data()+joint_values.size());
    robot_state->setVariablePositions(env_->getJointNames(), tmp_current_values);
    
    // Set (only) group joints from tesseract waypoint.
    const auto &joint_manip_names = env_->getManipulator(manip_)->getJointNames();
    std::vector<double> tmp_group_values(manip_state.data(), manip_state.data()+manip_state.size());
    robot_state->setVariablePositions(joint_manip_names, tmp_group_values);
}
