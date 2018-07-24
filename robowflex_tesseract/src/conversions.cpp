/* Author: Bryce Willey */

#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

#include <robowflex_tesseract/conversions.h>

#include <moveit_msgs/PlanningScene.h>

#include <tesseract_msgs/AttachableObject.h>
#include <tesseract_ros/ros_tesseract_utils.h>

using namespace robowflex;

tesseract::tesseract_ros::KDLEnvPtr hypercube::constructTesseractEnv(const robowflex::SceneConstPtr &scene,
                                                                     const robowflex::RobotConstPtr &robot)
{
    moveit_msgs::PlanningScene scene_msg = scene->getMessage();
    auto env = std::make_shared<tesseract::tesseract_ros::KDLEnv>();
    env->init(robot->getURDF(), robot->getSRDF());

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
            attached_body.parent_link_name = "base_link";
            attached_body.transform = scene->getFramePose(collision_object.header.frame_id) *
                                      scene->getFramePose("base_link").inverse();
        }

        env->attachBody(attached_body);
    }
    // TODO: fixed_frame_transforms?
    // TODO: actually use LinkPadding and LinkScales.
    // TODO: octomap
    // TODO: object_colors: should go somewhere in visual geometry colors.

    return env;
}
