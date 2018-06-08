#include "robowflex.h"

using namespace robowflex;

Scene::Scene(Robot &robot) : scene_(new planning_scene::PlanningScene(robot.getModel()))
{
    scene_->getPlanningSceneMsg(msg_);
}

robot_state::RobotState &Scene::getCurrentState()
{
    return scene_->getCurrentStateNonConst();
}

void Scene::addCollisionObject(const std::string &name, Geometry::Geometry &geometry, const std::string &base_frame,
                               const Eigen::Affine3d &pose)
{
    moveit_msgs::CollisionObject msg;
    msg.header.frame_id = base_frame;
    msg.id = name;

    if (geometry.isMesh())
    {
        msg.meshes.push_back(geometry.getMeshMsg());
        msg.mesh_poses.push_back(Geometry::poseEigenToMsg(pose));
    }
    else
    {
        msg.primitives.push_back(geometry.getSolidMsg());
        msg.primitive_poses.push_back(Geometry::poseEigenToMsg(pose));
    }

    msg.operation = moveit_msgs::CollisionObject::ADD;

    msg_.world.collision_objects.push_back(msg);
}

void Scene::removeCollisionObject(const std::string &name)
{
    for (auto obj = msg_.world.collision_objects.begin(); obj != msg_.world.collision_objects.begin(); ++obj)
    {
        if (obj->id == name)
        {
            msg_.world.collision_objects.erase(obj);
            return;
        }
    }
}
