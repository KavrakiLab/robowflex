/* Author: Bryce Willey */

// Updated by Carlos Quintero
// Added fromTesseractResToMoveitTraj

#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

#include <robowflex_tesseract/conversions.h>

#include <moveit_msgs/PlanningScene.h>

#include <tesseract_msgs/AttachableObject.h>
#include <tesseract_ros/ros_tesseract_utils.h>

using namespace robowflex;

tesseract::tesseract_ros::KDLEnvPtr hypercube::constructTesseractEnv(const robowflex::SceneConstPtr &scene,
                                                                     const robowflex::RobotConstPtr &robot, 
                                                                     const std::string &name
                                                                    )
{
    moveit_msgs::PlanningScene scene_msg = scene->getMessage();
    auto env = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
    env->init(robot->getURDF(), robot->getSRDF());
    env->setName(name);

    // Add all of the collision objects in the scene message.
    // So there's a name, robot_state, and robot_model_name. Going to ignore most
    // of that. Ignore the ACM, we get that from the robot.
    std::vector<std::string> link_names = robot->getModelConst()->getLinkModelNames();

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
        env->addAttachableObject(ao);

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
            const std::string &root_name = robot->getModelConst()->getRootLinkName();
            attached_body.parent_link_name = root_name;
            attached_body.transform = scene->getFramePose(collision_object.header.frame_id) *
                                      scene->getFramePose(root_name).inverse();
        }

        env->attachBody(attached_body);
    }
    // TODO: fixed_frame_transforms?
    // TODO: actually use LinkPadding and LinkScales.
    // TODO: octomap
    // TODO: object_colors: should go somewhere in visual geometry colors.

    return env;
}

void hypercube::fromTesseractResToMoveitTraj(const tesseract::tesseract_planning::PlannerResponse &response, 
                                             const tesseract::tesseract_ros::KDLEnvPtr &env, 
                                             const robowflex::RobotConstPtr &robot, 
                                             robot_trajectory::RobotTrajectoryPtr &trajectory)
{
    // TODO: Just pass the RobotModel instead of the whole robot?
    for (int i=0;i<response.trajectory.rows();i++)
    {
        // create a tmp state for every waypoint (initialize base joints by hand since they are not in env)
        auto tmpState = std::make_shared<moveit::core::RobotState>(robot->getModelConst());
        tmpState->setVariablePositions({"base_joint/x", "base_joint/y", "base_joint/theta"}, {0.0, 0.0, 0.0});
        
        // initialize it with the env start state (includes both group and non-group joints)
        double* rawJointValues = new double((int)env->getJointNames().size());
        rawJointValues = env->getCurrentJointValues().data();
        std::vector<double> tmpCurrentValues(rawJointValues, rawJointValues+(int)env->getJointNames().size());
        tmpState->setVariablePositions(env->getJointNames(), tmpCurrentValues);
        
        // set (only) group joints from tesseract response's ith row
        double* rawGroupValues = new double(response.joint_names.size());
        rawGroupValues = (double*) response.trajectory.row(i).transpose().data();
        std::vector<double> tmpGroupValues(rawGroupValues, rawGroupValues+(int)response.joint_names.size());
        tmpState->setVariablePositions(response.joint_names, tmpGroupValues);
        
        // add waypoint to trajectory
        trajectory->addSuffixWayPoint(tmpState, 0.5);
        //delete rawGroupValues;
        //delete rawJointValues;
    }
    
    /*std::cout << "First row of trajectory: " << std::endl;
    std::cout << response.trajectory.row(0) << std::endl;
    
    std::cout << "Number of waypoints (Tesseract): " << trajectory->getWayPointCount() << std::endl;
    for (int i=0;i<trajectory->getWayPointCount();i++)
    {
        double* val = trajectory->getWayPointPtr(i)->getVariablePositions();
        for (int j=0;j<trajectory->getWayPointPtr(i)->getVariableCount();j++)
            std::cout << val[j] << " ";
        std::cout << std::endl;
    }*/
}
