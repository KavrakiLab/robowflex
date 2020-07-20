/* Author: Carlos Quintero */

// Robowflex
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

// Tesseract
#include <tesseract_msgs/AttachableObject.h>
//#include <trajopt/file_write_callback.hpp>

// TrajOptPlanner
#include <robowflex_tesseract/trajopt_planner.h>
#include <robowflex_tesseract/conversions.h>

using namespace robowflex;

TrajOptPlanner::TrajOptPlanner(const RobotConstPtr &robot, const std::string &group_name)
  : robot_(robot), group_(group_name)
{
    env_ = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
    env_->init(robot_->getURDF(), robot_->getSRDF());
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);
}

tesseract::tesseract_planning::PlannerResponse TrajOptPlanner::plan(const SceneConstPtr &scene,
                                                                    const MotionRequestBuilderPtr &request)
{
    newEnvFromScene(scene);
    tesseract::tesseract_planning::PlannerResponse planning_response;
    
    // Extract start state from request.
    double* stateValues = new double((int)request->getStartConfiguration()->getVariableCount());
    stateValues = request->getStartConfiguration()->getVariablePositions();
    std::vector<double> values(stateValues, stateValues+(int)request->getStartConfiguration()->getVariableCount());       
    
    // Set the extracted start state into the Tesseract environment
    // (this is used by ProblemConstructionInfo (see basic_info.start_fixed).
    env_->setState(request->getStartConfiguration()->getVariableNames(), values);
    
    // Extract the goal state from the request
    double* rawGoalstateValues = new double((int)request->getGoalConfiguration()->getVariableCount());
    rawGoalstateValues = request->getGoalConfiguration()->getVariablePositions();
    auto groupJointNames = env_->getManipulator(group_)->getJointNames();
    std::vector<double> vectorGoalStateValues;
    for (const auto &envJointName : groupJointNames)
    {            
        int index = std::distance(request->getGoalConfiguration()->getVariableNames().begin(),
                                    std::find(request->getGoalConfiguration()->getVariableNames().begin(), 
                                              request->getGoalConfiguration()->getVariableNames().end(), 
                                              envJointName));
        vectorGoalStateValues.push_back(rawGoalstateValues[index]);
    }
    /*
    for (const auto &name: request->getGoalConfiguration()->getVariableNames())
        std::cout << name << ", ";
    std::cout << std::endl;
    
    for (int i=0;i<request->getGoalConfiguration()->getVariableCount();i++)
        std::cout << rawGoalstateValues[i] << ", ";
    std::cout << std::endl;
    
    for (const auto &val: vectorGoalStateValues)
        std::cout << val << ", ";
    std::cout << std::endl;
    */
    if (env_->checkInitialized())
    {                
        // Create the Tesseract Planner
        tesseract::tesseract_planning::TrajOptPlanner trajo_planner;
        
        // Fill in the problem construction info and initialization
        trajopt::ProblemConstructionInfo pci(env_);
        pci.kin = env_->getManipulator(group_);
        pci.basic_info.n_steps = numSteps_;//
        pci.basic_info.manip = group_;
        pci.basic_info.dt_lower_lim = 2;    // 1/most time
        pci.basic_info.dt_upper_lim = 100;  // 1/least time
        pci.basic_info.start_fixed = true;
        pci.basic_info.use_time = false;
        pci.init_info.type = trajopt::InitInfo::STATIONARY;
        pci.init_info.dt = 0.5;
                        
        // Add (continuous) collision costs to all waypoints in the trajectory
        if (true)
        {
            std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
            collision->name = "collision";
            collision->term_type = trajopt::TT_COST;
            collision->continuous = true;
            collision->first_step = 0;
            collision->last_step = pci.basic_info.n_steps - 1;
            collision->gap = 1;
            collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);
            pci.cost_infos.push_back(collision);
        }
        
        // Add joint velocity cost (without time) to penalize longer paths
        if (true)
        {
            std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
            jv->targets = std::vector<double>(pci.kin->numJoints(), 0.0);
            jv->coeffs = std::vector<double>(pci.kin->numJoints(), 5.0);
            jv->term_type = trajopt::TT_COST;
            jv->first_step = 0;
            jv->last_step = pci.basic_info.n_steps - 1;
            jv->name = "joint_velocity_cost";
            pci.cost_infos.push_back(jv);
        }
        
        // TODO: Joint velocity constraints to meet robot dynamics!
                        
        // Add joint pose cnt at the goalState
        if (true)
        {
            std::shared_ptr<trajopt::JointPosTermInfo> joint_pos_constraint(new trajopt::JointPosTermInfo);
            joint_pos_constraint->term_type = trajopt::TT_CNT;
            joint_pos_constraint->name = "pick_pose_cnt";
            joint_pos_constraint->coeffs = std::vector<double>(pci.kin->numJoints(), 5.0);
            joint_pos_constraint->targets = vectorGoalStateValues;
            joint_pos_constraint->first_step = pci.basic_info.n_steps-1;
            joint_pos_constraint->last_step = pci.basic_info.n_steps-1;
            pci.cnt_infos.push_back(joint_pos_constraint);
        }
        
        // Add a cost on the total time to complete the motion
        if (true)
        {
            std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
            time_cost->name = "time_cost";
            time_cost->coeff = 5.0;
            time_cost->limit = 0.0;
            time_cost->term_type = trajopt::TT_COST;
            pci.cost_infos.push_back(time_cost);
        }
        
        // TrajOpt problem and optimizer parameters
        trajopt::TrajOptProbPtr prob = ConstructProblem(pci);
        tesseract::tesseract_planning::TrajOptPlannerConfig config(prob);
        config.params.max_iter = 100;
        
        // Write planning results in file_output_pick.csv file
        //std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
        //if (file_write_cb)
        //{
        //    std::string path = ros::package::getPath("robowflex_datasets_fetch") + "/file_output_pick.csv";
        //    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
        //    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, prob));
        //}
        
        // Solve planning problem
        
        if (trajo_planner.solve(planning_response, config))
        {
            //std::cout << "Names: " << std::endl;
            //for (const auto &name : planning_response.joint_names)
            //    std::cout << name <<  ", ";
            //std::cout << std::endl;
            //auto currEnvState = env_->getCurrentJointValues();
            //std::cout << currEnvState << std::endl;
            //auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);
            trajectory_->clear();
            updateTrajFromTesseractResponse(planning_response);
        }
        
        //if (file_write_cb)
        //    stream_ptr->close();
    }
    else
        ROS_INFO("KDLEnv not initialized!!");
    return planning_response;
}

tesseract::tesseract_planning::PlannerResponse TrajOptPlanner::plan(const SceneConstPtr &scene, 
                                                                    const std::unordered_map<std::string, double> &start_state, 
                                                                    const Eigen::Isometry3d &goal_pose, 
                                                                    const std::string &end_effector)
{
    newEnvFromScene(scene);
    tesseract::tesseract_planning::PlannerResponse planning_response;
 
    // Set the extracted start state into the Tesseract environment
    // (this is used by ProblemConstructionInfo (see basic_info.start_fixed).
    env_->setState(start_state);
    
    // Create the Tesseract Planner
    tesseract::tesseract_planning::TrajOptPlanner trajo_planner;
    
    // Fill in the problem construction info and initialization
    trajopt::ProblemConstructionInfo pci(env_);
    pci.kin = env_->getManipulator(group_);
    pci.basic_info.n_steps = numSteps_;//
    pci.basic_info.manip = group_;
    pci.basic_info.dt_lower_lim = 2;    // 1/most time
    pci.basic_info.dt_upper_lim = 100;  // 1/least time
    pci.basic_info.start_fixed = true;
    pci.basic_info.use_time = false;
    pci.init_info.type = trajopt::InitInfo::STATIONARY;
    pci.init_info.dt = 0.5;
                    
    // Add (continuous) collision costs to all waypoints in the trajectory
    if (true)
    {
        std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
        collision->name = "collision";
        collision->term_type = trajopt::TT_COST;
        collision->continuous = true;
        collision->first_step = 0;
        collision->last_step = pci.basic_info.n_steps - 1;
        collision->gap = 1;
        collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);
        pci.cost_infos.push_back(collision);
    }
    
    // Add joint velocity cost (without time) to penalize longer paths
    if (true)
    {
        std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
        jv->targets = std::vector<double>(pci.kin->numJoints(), 0.0);
        jv->coeffs = std::vector<double>(pci.kin->numJoints(), 5.0);
        jv->term_type = trajopt::TT_COST;
        jv->first_step = 0;
        jv->last_step = pci.basic_info.n_steps - 1;
        jv->name = "joint_velocity_cost";
        pci.cost_infos.push_back(jv);
    }
    
    // Add cartesian pose cnt at the goal pose
    if (true)
    {
        Eigen::Quaterniond rotation(goal_pose.linear());
        std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
            std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
        pose_constraint->term_type = trajopt::TT_CNT;
        pose_constraint->link = end_effector;
        pose_constraint->timestep = numSteps_-1;
        pose_constraint->xyz = goal_pose.translation();

        pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
        pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
        pose_constraint->name = "pose_" + std::to_string(numSteps_-1);
        pci.cnt_infos.push_back(pose_constraint);
    }
    
      // Add a cost on the total time to complete the pick
    if (true)
    {
        std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
        time_cost->name = "time_cost";
        time_cost->coeff = 5.0;
        time_cost->limit = 0.0;
        time_cost->term_type = trajopt::TT_COST;
        pci.cost_infos.push_back(time_cost);
    }
    
    // TrajOpt problem and optimizer parameters
    trajopt::TrajOptProbPtr prob = ConstructProblem(pci);
    tesseract::tesseract_planning::TrajOptPlannerConfig config(prob);
    config.params.max_iter = 100;
    
    // Write planning results in file_output_pick.csv file
    //std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
    //if (file_write_cb)
    //{
    //    std::string path = ros::package::getPath("robowflex_datasets_fetch") + "/file_output_pick.csv";
    //    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    //    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, prob));
    //}
    
    // Solve planning problem
    
    if (trajo_planner.solve(planning_response, config))
    {
        //std::cout << "Names: " << std::endl;
        //for (const auto &name : planning_response.joint_names)
        //    std::cout << name <<  ", ";
        //std::cout << std::endl;
        //auto currEnvState = env_->getCurrentJointValues();
        //std::cout << currEnvState << std::endl;
        //auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group_);
        trajectory_->clear();
        updateTrajFromTesseractResponse(planning_response);
    }
    return planning_response;
}

robot_trajectory::RobotTrajectoryPtr TrajOptPlanner::getTrajectory()
{
    return trajectory_;
}

bool TrajOptPlanner::newEnvFromScene(const robowflex::SceneConstPtr &scene)
{
    //TODO: Set a name for the scene?
    env_->clearAttachableObjects();
    env_->clearAttachedBodies();
    
    moveit_msgs::PlanningScene scene_msg = scene->getMessage();
    
    // Add all of the collision objects in the scene message.
    // So there's a name, robot_state, and robot_model_name. Going to ignore most
    // of that. Ignore the ACM, we get that from the robot.
    std::vector<std::string> link_names = robot_->getModelConst()->getLinkModelNames();

    // Add all of the collision objects in the scene message.
    for (const auto &collision_object : scene_msg.world.collision_objects)
    {
        tesseract_msgs::AttachableObject obj;
        obj.name = collision_object.id;
        obj.operation = tesseract_msgs::AttachableObject::ADD;

        // Add all of the objects.
        for (const shape_msgs::SolidPrimitive &primitive : collision_object.primitives)
        {
            obj.visual.primitives.emplace_back(primitive);
            obj.collision.primitives.emplace_back(primitive);
            std_msgs::Int32 cot;
            cot.data = tesseract::CollisionObjectTypes::CollisionObjectType::UseShapeType;
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

        auto ao = std::make_shared<tesseract::AttachableObject>();
        tesseract::tesseract_ros::attachableObjectMsgToAttachableObject(ao, obj);
        env_->addAttachableObject(ao);

        // Determine where this collision object should be attached to the robot.
        // Tesseract doesn't allow objects to be just in the scene.
        tesseract::AttachedBodyInfo attached_body;
        attached_body.object_name = collision_object.id;
        auto it = std::find(link_names.begin(), link_names.end(), collision_object.header.frame_id);
        if (it != link_names.end())
        {
            // If the header frame_id is a link name, directly attach it.
            attached_body.parent_link_name = *it;
            attached_body.transform.setIdentity();
        }
        else
        {
            // The header frame_id is somewhere else in the scene.
            // Attach it to the base link, but make the transform go from frame_id to
            // world to base_link.
            const std::string &root_name = robot_->getModelConst()->getRootLinkName();
            attached_body.parent_link_name = root_name;
            attached_body.transform = scene->getFramePose(collision_object.header.frame_id) *
                                      scene->getFramePose(root_name).inverse();
        }

        env_->attachBody(attached_body);
    }
    
    return false;
}

void TrajOptPlanner::updateTrajFromTesseractResponse(const tesseract::tesseract_planning::PlannerResponse &response)
{
    for (int i=0;i<response.trajectory.rows();i++)
    {
        // create a tmp state for every waypoint (initialize base joints by hand since they are not in env)
        auto tmpState = std::make_shared<moveit::core::RobotState>(robot_->getModelConst());
        //tmpState->setVariablePositions({"base_joint/x", "base_joint/y", "base_joint/theta"}, {0.0, 0.0, 0.0});
        
        // initialize it with the env start state (includes both group and non-group joints)
        double* rawJointValues = new double((int)env_->getJointNames().size());
        rawJointValues = env_->getCurrentJointValues().data();
        std::vector<double> tmpCurrentValues(rawJointValues, rawJointValues+(int)env_->getJointNames().size());
        tmpState->setVariablePositions(env_->getJointNames(), tmpCurrentValues);
        
        // set (only) group joints from tesseract response's ith row
        double* rawGroupValues = new double(response.joint_names.size());
        rawGroupValues = (double*) response.trajectory.row(i).transpose().data();
        std::vector<double> tmpGroupValues(rawGroupValues, rawGroupValues+(int)response.joint_names.size());
        tmpState->setVariablePositions(response.joint_names, tmpGroupValues);
        
        // add waypoint to trajectory
        trajectory_->addSuffixWayPoint(tmpState, 0.5);
    }
}
