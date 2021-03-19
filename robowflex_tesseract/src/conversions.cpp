/* Author: Carlos Quintero, Bryce Willey */

// Tesseract
#include <tesseract_ros/ros_tesseract_utils.h>

// Conversions
#include <robowflex_tesseract/conversions.h>

using namespace robowflex;

bool hypercube::sceneToTesseractEnv(const robowflex::SceneConstPtr &scene,
                                    tesseract::tesseract_ros::KDLEnvPtr &env)
{
    if (env->checkInitialized())
    {
        // Clear environment from scene objects.
        env->clearAttachableObjects();
        env->clearAttachedBodies();

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
            env->addAttachableObject(ao);

            // Attach the object to the environment (to a parent frame).
            tesseract::AttachedBodyInfo attached_body_info;
            attached_body_info.object_name = collision_object.id;
            attached_body_info.parent_link_name = env->getRootLinkName();
            attached_body_info.transform = Eigen::Isometry3d::Identity();
            env->attachBody(attached_body_info);
        }

        return true;
    }

    ROS_ERROR("Tesseract environment not initialized");
    return false;
    // TODO: fixed_frame_transforms?
    // TODO: actually use LinkPadding and LinkScales.
    // TODO: octomap
    // TODO: object_colors: should go somewhere in visual geometry colors.
}

bool hypercube::addAttachedBodiesToTesseractEnv(const robot_state::RobotStatePtr &state,
                                                tesseract::tesseract_ros::KDLEnvPtr &env)
{
    // Find bodies attached to the robot scratch state.
    std::vector<const moveit::core::AttachedBody *> attached_bodies;
    state->getAttachedBodies(attached_bodies);
    for (const auto &link : attached_bodies)
    {
        // Get shapes and poses of this attached body.
        const auto &shapes = link->getShapes();
        int num_shapes = shapes.size();
        const auto &poses = link->getFixedTransforms();
        // const auto global_poses = link->getGlobalCollisionBodyTransforms();

        // Declare an attachable object for this attached body.
        tesseract_msgs::AttachableObject obj;
        obj.name = link->getName();
        obj.operation = tesseract_msgs::AttachableObject::ADD;

        for (int i = 0; i < num_shapes; ++i)
        {
            // Add shape primitive to visual and collision for this shape.
            shapes::ShapeMsg shape_msg;
            shapes::constructMsgFromShape(shapes[i].get(), shape_msg);
            auto shape_primitive = boost::get<shape_msgs::SolidPrimitive>(shape_msg);
            obj.visual.primitives.emplace_back(shape_primitive);
            obj.collision.primitives.emplace_back(shape_primitive);

            std_msgs::Int32 cot;
            cot.data = tesseract::CollisionObjectType::UseShapeType;
            obj.collision.primitive_collision_object_types.emplace_back(cot);

            // Add primitive poses for this shape
            geometry_msgs::Pose primitive_pose;
            tf::poseEigenToMsg(poses[i], primitive_pose);
            obj.visual.primitive_poses.emplace_back(primitive_pose);
            obj.collision.primitive_poses.emplace_back(primitive_pose);
        }

        auto ao = std::make_shared<tesseract::AttachableObject>();
        tesseract::tesseract_ros::attachableObjectMsgToAttachableObject(ao, obj);
        env->addAttachableObject(ao);

        // Attach the object to the environment (to a parent frame).
        tesseract::AttachedBodyInfo attached_body_info;
        attached_body_info.object_name = link->getName();
        attached_body_info.parent_link_name = link->getAttachedLinkName();
        attached_body_info.transform = Eigen::Isometry3d::Identity();
        env->attachBody(attached_body_info);
    }

    return true;
}

void hypercube::robotStateToManipState(const robot_state::RobotStatePtr &robot_state,
                                       const std::vector<std::string> &manip_joint_names,
                                       std::vector<double> &manip_joint_values)
{
    manip_joint_values.clear();
    manip_joint_values.resize(0);

    // Extract the state to a raw vector.
    double *raw_state_values = robot_state->getVariablePositions();

    // Loop over manip joints and add their values to goal_state in the correct order.
    const auto &robot_state_joint_names = robot_state->getVariableNames();
    for (const auto &joint_name : manip_joint_names)
    {
        int index = std::distance(
            robot_state_joint_names.begin(),
            std::find(robot_state_joint_names.begin(), robot_state_joint_names.end(), joint_name));
        manip_joint_values.push_back(raw_state_values[index]);
    }
}

void hypercube::manipStateToRobotState(const Eigen::Ref<const Eigen::VectorXd> &manip_state,
                                       const std::string &manip,
                                       const tesseract::tesseract_ros::KDLEnvPtr &env,
                                       robot_state::RobotStatePtr &robot_state)
{
    // Initialize it with the env state (includes both group and non-group joints).
    const auto &joint_values = env->getCurrentJointValues();
    std::vector<double> tmp_current_values(joint_values.data(), joint_values.data() + joint_values.size());
    robot_state->setVariablePositions(env->getJointNames(), tmp_current_values);

    // Set (only) group joints from tesseract waypoint.
    const auto &joint_manip_names = env->getManipulator(manip)->getJointNames();
    std::vector<double> tmp_group_values(manip_state.data(), manip_state.data() + manip_state.size());
    robot_state->setVariablePositions(joint_manip_names, tmp_group_values);
}

void hypercube::manipTesseractTrajToRobotTraj(const tesseract::TrajArray &tesseract_traj,
                                              const robot_state::RobotStatePtr &ref_state,
                                              const std::string &manip,
                                              const tesseract::tesseract_ros::KDLEnvPtr &env,
                                              robot_trajectory::RobotTrajectoryPtr &trajectory)
{
    const robot_state::RobotState &copy = *ref_state;
    trajectory->clear();

    for (int i = 0; i < tesseract_traj.rows(); i++)
    {
        // Create a tmp state for every waypoint.
        auto tmp_state = std::make_shared<robot_state::RobotState>(copy);

        // Transform tesseract manip ith waypoint to robot state.
        manipStateToRobotState(tesseract_traj.row(i), manip, env, tmp_state);

        // Add waypoint to trajectory.
        trajectory->addSuffixWayPoint(tmp_state, 0.0);
    }
}

void hypercube::robotTrajToManipTesseractTraj(const robot_trajectory::RobotTrajectoryPtr &robot_traj,
                                              const std::string &manip,
                                              const tesseract::tesseract_ros::KDLEnvPtr &env,
                                              tesseract::TrajArray &trajectory)
{
    trajectory.resize(0, 0);
    trajectory.resize(robot_traj->getWayPointCount(), robot_traj->getGroup()->getVariableCount());

    for (unsigned int i = 0; i < robot_traj->getWayPointCount(); ++i)
    {
        // Transform each traj waypoint to tesseract manip state.
        std::vector<double> manip_joint_values;
        robotStateToManipState(robot_traj->getWayPointPtr(i), env->getManipulator(manip)->getJointNames(),
                               manip_joint_values);

        // Push manip state to manip tesseract trajectory .
        trajectory.row(i) = Eigen::VectorXd::Map(manip_joint_values.data(), manip_joint_values.size());
    }
}
