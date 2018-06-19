#include <robowflex_library/robowflex.h>

using namespace robowflex;

Scene::Scene(Robot &robot) : scene_(new planning_scene::PlanningScene(robot.getModel()))
{
}

Scene::Scene(const Scene &other) : scene_(other.getSceneConst())
{
}

void Scene::operator=(const Scene &other)
{
    scene_ = other.getSceneConst();
}

moveit_msgs::PlanningScene Scene::getMessage() const
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

void Scene::updateCollisionObject(const std::string &name, const Geometry &geometry,
                                  const Eigen::Affine3d &pose)
{
    auto &world = scene_->getWorldNonConst();
    if (world->hasObject(name))
    {
        if (!world->moveShapeInObject(name, geometry.getShape(), pose))
            world->removeObject(name);
        else
            return;
    }

    world->addToObject(name, geometry.getShape(), pose);
}

void Scene::removeCollisionObject(const std::string &name)
{
    scene_->getWorldNonConst()->removeObject(name);
}

Eigen::Affine3d Scene::getObjectPose(const std::string &name)
{
    auto &world = scene_->getWorldNonConst();
    const auto &obj = world->getObject(name);
    if (obj)
        return obj->shape_poses_[0];

    return Eigen::Affine3d::Identity();
}

bool Scene::attachObject(const std::string &name)
{
    const auto &robot = scene_->getCurrentState().getRobotModel();
    const auto &ee = robot->getEndEffectors();

    // One end-effector
    if (ee.size() == 1)
    {
        const auto &links = ee[0]->getLinkModelNames();
        return attachObject(name, links[0], links);
    }

    return false;
}

bool Scene::attachObject(const std::string &name, const std::string &ee_link,
                         const std::vector<std::string> &touch_links)
{
    auto &world = scene_->getWorldNonConst();
    if (!world->hasObject(name))
    {
        ROS_ERROR("World does not have object `%s`", name.c_str());
        return false;
    }

    auto &robot = scene_->getCurrentStateNonConst();
    const auto &obj = world->getObject(name);

    if (!obj)
    {
        ROS_ERROR("Could not get object `%s`", name.c_str());
        return false;
    }

    if (!world->removeObject(name))
    {
        ROS_ERROR("Could not remove object `%s`", name.c_str());
        return false;
    }

    robot.attachBody(name, obj->shapes_, obj->shape_poses_, touch_links, ee_link);
    return true;
}

bool Scene::detachObject(const std::string &name)
{
    auto &robot = scene_->getCurrentStateNonConst();
    auto &world = scene_->getWorldNonConst();
    auto body = robot.getAttachedBody(name);
    if (!body)
    {
        ROS_ERROR("Robot does not have attached object `%s`", name.c_str());
        return false;
    }

    world->addToObject(name, body->getShapes(), body->getFixedTransforms());
}

bool Scene::toYAMLFile(const std::string &file)
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);

    return IO::messageToYAMLFile(msg, file);
}

bool Scene::fromYAMLFile(const std::string &file)
{
    moveit_msgs::PlanningScene msg;
    if (!IO::YAMLFileToMessage(msg, file))
        return false;

    scene_->setPlanningSceneMsg(msg);
    return true;
}
