#include "robowflex.h"

using namespace robowflex;

Scene::Scene(Robot &robot) : scene_(new planning_scene::PlanningScene(robot.getModel()))
{
    scene_->getPlanningSceneMsg(msg_);
}

moveit_msgs::PlanningScene Scene::getMessage()
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);
    return msg;
}

robot_state::RobotState &Scene::getCurrentState()
{
    return scene_->getCurrentStateNonConst();
}

collision_detection::AllowedCollisionMatrix &Scene::getACM()
{
    return scene_->getAllowedCollisionMatrixNonConst();
}

void Scene::addCollisionObject(const std::string &name, const Geometry &geometry, const Eigen::Affine3d &pose)
{
    scene_->getWorldNonConst()->addToObject(name, geometry.getShape(), pose);
}

void Scene::removeCollisionObject(const std::string &name)
{
    scene_->getWorldNonConst()->removeObject(name);
}
