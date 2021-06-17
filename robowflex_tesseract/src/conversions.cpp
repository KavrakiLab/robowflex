/* Author: Carlos Quintero, Bryce Willey */

// MoveIt!
#include <moveit/robot_state/conversions.h>

// Robowflex
#include <robowflex_library/log.h>

// Tesseract
#include <tesseract_ros/ros_tesseract_utils.h>

// Conversions
#include <robowflex_tesseract/conversions.h>

using namespace robowflex;

bool hypercube::sceneToTesseractEnv(const robowflex::SceneConstPtr &scene,
                                    tesseract::tesseract_ros::KDLEnvPtr env)
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
                std_msgs::Int32 cot;
                cot.data = tesseract::CollisionObjectType::ConvexHull;
                obj.collision.mesh_collision_object_types.emplace_back(cot);
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

        // Add octomap information.
        if (scene_msg.world.octomap.octomap.data.size() > 0)
        {
            const std::string &name = "octomap";
            tesseract_msgs::AttachableObject obj;
            obj.name = name;
            obj.operation = tesseract_msgs::AttachableObject::ADD;
            obj.collision.octomaps.emplace_back(scene_msg.world.octomap.octomap);
            obj.collision.octomap_poses.emplace_back(scene_msg.world.octomap.origin);
            std_msgs::Int32 cot;
            cot.data = tesseract::CollisionObjectType::UseShapeType;
            obj.collision.octomap_collision_object_types.emplace_back(cot);

            auto ao = std::make_shared<tesseract::AttachableObject>();
            tesseract::tesseract_ros::attachableObjectMsgToAttachableObject(ao, obj);
            env->addAttachableObject(ao);

            tesseract::AttachedBodyInfo attached_body_info;
            attached_body_info.object_name = name;
            attached_body_info.parent_link_name = scene_msg.world.octomap.header.frame_id;
            attached_body_info.transform = Eigen::Isometry3d::Identity();
            env->attachBody(attached_body_info);
        }

        return true;
    }

    RBX_ERROR("Tesseract environment not initialized");

    return false;
}

bool hypercube::addAttachedBodiesToTesseractEnv(const robot_state::RobotStatePtr &state,
                                                tesseract::tesseract_ros::KDLEnvPtr env)
{
    moveit_msgs::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(*state, state_msg);

    std_msgs::Int32 cotp;
    cotp.data = tesseract::CollisionObjectType::UseShapeType;
    std_msgs::Int32 cotm;
    cotm.data = tesseract::CollisionObjectType::ConvexHull;

    for (const auto &co : state_msg.attached_collision_objects)
    {
        // Declare an attachable object for this attached body.
        tesseract_msgs::AttachableObject obj;
        obj.name = co.object.id;
        obj.operation = tesseract_msgs::AttachableObject::ADD;

        // Add visual object.
        obj.visual.primitives = co.object.primitives;
        obj.visual.primitive_poses = co.object.primitive_poses;
        obj.visual.meshes = co.object.meshes;
        obj.visual.mesh_poses = co.object.mesh_poses;
        obj.visual.planes = co.object.planes;
        obj.visual.plane_poses = co.object.plane_poses;

        // Add collision object.
        obj.collision.primitives = co.object.primitives;
        obj.collision.primitive_poses = co.object.primitive_poses;
        obj.collision.primitive_collision_object_types.resize(co.object.primitives.size());
        std::fill(obj.collision.primitive_collision_object_types.begin(),
                  obj.collision.primitive_collision_object_types.end(), cotp);
        obj.collision.meshes = co.object.meshes;
        obj.collision.mesh_poses = co.object.mesh_poses;
        obj.collision.mesh_collision_object_types.resize(co.object.meshes.size());
        std::fill(obj.collision.mesh_collision_object_types.begin(),
                  obj.collision.mesh_collision_object_types.end(), cotm);
        obj.collision.planes = co.object.planes;
        obj.collision.plane_poses = co.object.plane_poses;

        // Create Tesseract attachable object.
        auto ao = std::make_shared<tesseract::AttachableObject>();
        tesseract::tesseract_ros::attachableObjectMsgToAttachableObject(ao, obj);
        env->addAttachableObject(ao);

        // Attach the object to the environment (to a parent frame).
        tesseract::AttachedBodyInfo attached_body_info;
        attached_body_info.object_name = co.object.id;
        attached_body_info.parent_link_name = co.link_name;
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
                                       robot_state::RobotStatePtr robot_state)
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
                                              robot_trajectory::RobotTrajectoryPtr trajectory)
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
