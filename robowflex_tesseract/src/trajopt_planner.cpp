/* Author: Carlos Quintero */

// Robowflex
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

// Tesseract
#include <tesseract_msgs/AttachableObject.h>

// TrajOptPlanner
#include <robowflex_tesseract/trajopt_planner.h>
#include <trajopt/plot_callback.hpp>

using namespace robowflex;

TrajOptPlanner::TrajOptPlanner(const RobotConstPtr &robot, const std::string &group_name, const std::string &manip)
  : robot_(robot), group_(group_name), manip_(manip)
{
    env_ = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
    env_->init(robot_->getURDF(), robot_->getSRDF());
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);
}

tesseract::tesseract_planning::PlannerResponse TrajOptPlanner::plan(const SceneConstPtr &scene,
                                                                    const MotionRequestBuilderPtr &request)
{
    // Crate the tesseract environment from the scene.
    createTesseractEnvFromScene(scene);
    
    // Fill in the problem construction info and initialization
    auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
    problemConstructionInfo(pci);
    
    // Add start state from request.
    addStartState(request, pci);
    
    // Add collision costs to all waypoints in the trajectory
    addCollisionAvoidance(pci);
        
    // Add goal state from request
    addGoalState(request, pci);
   
    // TrajOpt problem and optimizer parameters
    trajopt::TrajOptProbPtr prob = ConstructProblem(*pci);
    tesseract::tesseract_planning::TrajOptPlannerConfig config(prob);
    config.params.max_iter = 100;
    if (file_write_cb_)
        config.callbacks.push_back(trajopt::WriteCallback(stream_ptr_, prob));

     // Create the Tesseract Planner and response
    tesseract::tesseract_planning::TrajOptPlanner trajo_planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;
    
    // Solve planning problem
    if (trajo_planner.solve(planning_response, config))
    {
        // Clean previous trajectory and update it with new one
        trajectory_->clear();
        updateTrajFromTesseractRes(planning_response);

        // Write optimization results in file.
        if (file_write_cb_)
            stream_ptr_->close();
        
        std::cout << planning_response.trajectory << '\n';
        std::cout << planning_response.status_description << '\n';
    }
    return planning_response;
}

tesseract::tesseract_planning::PlannerResponse TrajOptPlanner::plan(const SceneConstPtr &scene, 
                                                                    const std::unordered_map<std::string, double> &start_state, 
                                                                    const Eigen::Isometry3d &goal_pose, 
                                                                    const std::string &link)
{
    // Crate the tesseract environment from the scene.
    createTesseractEnvFromScene(scene);
    
    // Fill in the problem construction info and initialization
    auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
    problemConstructionInfo(pci);
    
    // Add start state
    addStartState(start_state, pci);
    
    // Add collision costs to all waypoints in the trajectory
    addCollisionAvoidance(pci);
    
    // Add goal pose for link
    addGoalPose(goal_pose, link, pci);
    
    // TrajOpt problem and optimizer parameters
    trajopt::TrajOptProbPtr prob = ConstructProblem(*pci);
    tesseract::tesseract_planning::TrajOptPlannerConfig config(prob);
    config.params.max_iter = 100;
    if (file_write_cb_)
        config.callbacks.push_back(trajopt::WriteCallback(stream_ptr_, prob));
    
    // Create the Tesseract Planner and response
    tesseract::tesseract_planning::TrajOptPlanner trajo_planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;
    
    // Solve planning problem
    if (trajo_planner.solve(planning_response, config))
    {
        // Clean previous trajectory and update it with new one
        trajectory_->clear();
        updateTrajFromTesseractRes(planning_response);

        // Write optimization results in file.
        if (file_write_cb_)
            stream_ptr_->close();
        
        std::cout << planning_response.trajectory << '\n';
        std::cout << planning_response.status_description << '\n';
    }
    return planning_response;
}

void TrajOptPlanner::setWriteFile(bool file_write_cb, const std::string &file_path)
{
    file_write_cb_ = file_write_cb;
    file_path_ = file_path + "/file_output.csv";
    stream_ptr_ = std::make_shared<std::ofstream>();
    stream_ptr_->open(file_path_, std::ofstream::out | std::ofstream::trunc);
}

bool TrajOptPlanner::createTesseractEnvFromScene(const robowflex::SceneConstPtr &scene)
{
    //TODO: Set a name for the scene?
    if (env_->checkInitialized())
    {
        // Clear environment from scene objects.
        env_->clearAttachableObjects();
        env_->clearAttachedBodies();
        
        // Loop over collision objects in the scene.
        moveit_msgs::PlanningScene scene_msg = scene->getMessage();
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
            const std::string &root_name = robot_->getModelConst()->getRootLinkName();
            attached_body_info.parent_link_name = root_name;
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
        auto tmp_state = std::make_shared<moveit::core::RobotState>(robot_->getModelConst());
        
        // Initialize it with the env start state (includes both group and non-group joints).
        double* raw_joint_values = new double((int)env_->getJointNames().size());
        raw_joint_values = env_->getCurrentJointValues().data();
        std::vector<double> tmp_current_values(raw_joint_values, raw_joint_values+(int)env_->getJointNames().size());
        tmp_state->setVariablePositions(env_->getJointNames(), tmp_current_values);
        
        // Set (only) group joints from tesseract response's ith row.
        double* raw_group_values = new double(response.joint_names.size());
        raw_group_values = (double*) response.trajectory.row(i).transpose().data();
        std::vector<double> tmp_group_values(raw_group_values, raw_group_values+(int)response.joint_names.size());
        tmp_state->setVariablePositions(response.joint_names, tmp_group_values);
        
        // Add waypoint to trajectory.
        trajectory_->addSuffixWayPoint(tmp_state, 0.5);
    }
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
    pci->init_info.type = trajopt::InitInfo::STATIONARY;
    //pci.init_info.type = trajopt::InitInfo::JOINT_INTERPOLATED;//It requires end pose in init_info.data (Eigen::VectorXd)
    pci->init_info.dt = 0.5;
    
    // Add joint velocity cost (without time) to penalize longer paths.
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
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
    std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = cont_cc_;
    collision->first_step = 0;
    collision->last_step = num_waypoints_ - 1;
    collision->gap = 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci->basic_info.n_steps, 0.025, 45);
    pci->cost_infos.push_back(collision);
}

void TrajOptPlanner::addGoalPose(const Eigen::Isometry3d &goal_pose, const std::string &link, 
                                 std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    Eigen::Quaterniond rotation(goal_pose.linear());
    std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
        std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = link;
    pose_constraint->timestep = num_waypoints_-1;
    pose_constraint->xyz = goal_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(num_waypoints_-1);
    pci->cnt_infos.push_back(pose_constraint);
}

void TrajOptPlanner::addGoalState(const std::vector<double> goal_state,
                                  std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    std::shared_ptr<trajopt::JointPosTermInfo> joint_pos_constraint(new trajopt::JointPosTermInfo);
    joint_pos_constraint->term_type = trajopt::TT_CNT;
    joint_pos_constraint->name = "goal_state_cnt";
    joint_pos_constraint->coeffs = std::vector<double>(pci->kin->numJoints(), 5.0);
    joint_pos_constraint->targets = goal_state;
    joint_pos_constraint->first_step = num_waypoints_-1;
    joint_pos_constraint->last_step = num_waypoints_-1;
    pci->cnt_infos.push_back(joint_pos_constraint);
}

void TrajOptPlanner::addGoalState(const MotionRequestBuilderPtr &request, 
                                  std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Extract the goal state from the request
    double* raw_goal_state_values = new double((int)request->getGoalConfiguration()->getVariableCount());
    raw_goal_state_values = request->getGoalConfiguration()->getVariablePositions();
    auto group_joint_names = env_->getManipulator(manip_)->getJointNames();
    std::vector<double> vectorGoalStateValues;
    for (const auto &joint_name : group_joint_names)
    {            
        int index = std::distance(request->getGoalConfiguration()->getVariableNames().begin(),
                                  std::find(request->getGoalConfiguration()->getVariableNames().begin(), 
                                            request->getGoalConfiguration()->getVariableNames().end(), 
                                            joint_name));
        vectorGoalStateValues.push_back(raw_goal_state_values[index]);
    }
    addGoalState(vectorGoalStateValues, pci);
}

void TrajOptPlanner::addStartState(const std::unordered_map<std::string, double> &start_state, 
                                   std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Set the start_state into the Tesseract environment
    env_->setState(start_state);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::addStartState(const MotionRequestBuilderPtr &request, 
                                   std::shared_ptr<trajopt::ProblemConstructionInfo> &pci)
{
    // Extract start state from request.
    double* state_values = new double((int)request->getStartConfiguration()->getVariableCount());
    state_values = request->getStartConfiguration()->getVariablePositions();
    std::vector<double> values(state_values, state_values+(int)request->getStartConfiguration()->getVariableCount());   
    
    // Set the start_state into the Tesseract environment
    env_->setState(request->getStartConfiguration()->getVariableNames(), values);
    pci->basic_info.start_fixed = true;
}

void TrajOptPlanner::printTesseractEnvLinks()
{
    std::cout << "Env links: " << std::endl;
    for (const auto &name : env_->getLinkNames())
        std::cout << name << std::endl;
    std::cout << std::endl;
}

void TrajOptPlanner::printManipulatorLinks()
{
    std::cout << "Manipulator links: " << std::endl;
    for (const auto &name : env_->getManipulator(manip_)->getLinkNames())
        std::cout << name << std::endl;
}

void TrajOptPlanner::printManipulatorJoints()
{
    std::cout << "Manipulator joints: " << std::endl;
    for (const auto &name : env_->getManipulator(manip_)->getJointNames())
        std::cout << name << std::endl;
}
