/* Author: Zachary Kingston */

#include <robowflex_library/macros.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/openrave.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>

#include <moveit/collision_detection/collision_plugin.h>
#include <pluginlib/class_loader.h>

namespace robowflex
{
    /** \brief The actual plugin loader for collision plugins.
     *  Heavily inspired by code in moveit_ros/planning. */
    class Scene::CollisionPluginLoader
    {
        /** \brief The pluginlib loader for collision detection plugins.
         */
        using PluginLoader = pluginlib::ClassLoader<collision_detection::CollisionPlugin>;

    public:
        /** \brief Constructor. Attempts to create the pluginlib loader for collision plugins.
         */
        CollisionPluginLoader()
        {
            try
            {
                loader_.reset(new PluginLoader("moveit_core", "collision_detection::CollisionPlugin"));
            }
            catch (pluginlib::PluginlibException &e)
            {
                ROS_ERROR("Unable to construct collision plugin loader. Error: %s", e.what());
            }
        }

        /** \brief Attempts to load the collision detector plugin by the given \a name.
         *  Saves the plugin in an internal map and returns it if found.
         *  \param[in] name Name of the plugin to load.
         *  \return An allocated collision plugin.
         */
        collision_detection::CollisionPluginPtr load(const std::string &name)
        {
            collision_detection::CollisionPluginPtr plugin;

            try
            {
                plugin.reset(loader_->createUnmanagedInstance(name));
                plugins_[name] = plugin;
            }
            catch (pluginlib::PluginlibException &ex)
            {
                ROS_ERROR_STREAM("Exception while loading " << name << ": " << ex.what());
            }

            return plugin;
        }

        /** \brief Loads a collision detector into a planning scene instance.
         *  \param[in] name the plugin name
         *  \param[in] scene the planning scene instance.
         *  \param[in] exclusive If true, the new collision detector is the only one.
         *  \return True if the new collision detector is added to the scene.
         */
        bool activate(const std::string &name, const planning_scene::PlanningScenePtr &scene, bool exclusive)
        {
            auto it = plugins_.find(name);
            if (it == plugins_.end())
            {
                collision_detection::CollisionPluginPtr plugin = load(name);
                if (plugin)
                    return plugin->initialize(scene, exclusive);
            }
            else if (it->second)
                return it->second->initialize(scene, exclusive);

            return false;
        }

    private:
        std::shared_ptr<PluginLoader> loader_;                                    // The pluginlib loader.
        std::map<std::string, collision_detection::CollisionPluginPtr> plugins_;  ///< Loaded plugins.
    };
}  // namespace robowflex

using namespace robowflex;

Scene::Scene(const RobotConstPtr &robot)
  : loader_(new CollisionPluginLoader()), scene_(new planning_scene::PlanningScene(robot->getModelConst()))
{
}

Scene::Scene(const robot_model::RobotModelConstPtr &robot)
  : loader_(new CollisionPluginLoader()), scene_(new planning_scene::PlanningScene(robot))
{
}

Scene::Scene(const Scene &other) : loader_(new CollisionPluginLoader()), scene_(other.getSceneConst())
{
}

void Scene::operator=(const Scene &other)
{
    incrementVersion();
    scene_ = other.getSceneConst();
}

ScenePtr Scene::deepCopy() const
{
    auto scene = std::make_shared<Scene>(scene_->getRobotModel());
    scene->useMessage(getMessage());

    return scene;
}

const planning_scene::PlanningScenePtr &Scene::getSceneConst() const
{
    return scene_;
}

planning_scene::PlanningScenePtr &Scene::getScene()
{
    incrementVersion();
    return scene_;
}

moveit_msgs::PlanningScene Scene::getMessage() const
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);
    return msg;
}

robot_state::RobotState &Scene::getCurrentState()
{
    incrementVersion();
    return scene_->getCurrentStateNonConst();
}

const robot_state::RobotState &Scene::getCurrentStateConst() const
{
    return scene_->getCurrentState();
}

collision_detection::AllowedCollisionMatrix &Scene::getACM()
{
    incrementVersion();
    return scene_->getAllowedCollisionMatrixNonConst();
}

const collision_detection::AllowedCollisionMatrix &Scene::getACMConst() const
{
    return scene_->getAllowedCollisionMatrix();
}

void Scene::useMessage(const moveit_msgs::PlanningScene &msg, bool diff)
{
    incrementVersion();

    if (!diff)
        scene_->setPlanningSceneMsg(msg);
    else
        scene_->setPlanningSceneDiffMsg(msg);
}

void Scene::updateCollisionObject(const std::string &name, const GeometryConstPtr &geometry,
                                  const RobotPose &pose)
{
    incrementVersion();

    auto &world = scene_->getWorldNonConst();
    if (world->hasObject(name))
    {
        if (!world->moveShapeInObject(name, geometry->getShape(), pose))
            world->removeObject(name);
        else
            return;
    }

    world->addToObject(name, geometry->getShape(), pose);
}

std::vector<std::string> Scene::getCollisionObjects() const
{
    auto &world = scene_->getWorld();
    return world->getObjectIds();
}

GeometryPtr Scene::getObjectGeometry(const std::string &name) const
{
    auto &world = scene_->getWorld();

    const auto &obj = world->getObject(name);
    if (obj)
        return std::make_shared<Geometry>(*obj->shapes_[0]);

    ROS_WARN("Object %s does not exist in scene!", name.c_str());
    return nullptr;
}

void Scene::removeCollisionObject(const std::string &name)
{
    scene_->getWorldNonConst()->removeObject(name);
}

RobotPose Scene::getObjectPose(const std::string &name) const
{
    auto &world = scene_->getWorldNonConst();
    const auto &obj = world->getObject(name);
    if (obj)
        return obj->shape_poses_[0];

    return RobotPose::Identity();
}

bool Scene::moveObjectGlobal(const std::string &name, const RobotPose &transform)
{
    incrementVersion();

    bool success = false;
#if ROBOWFLEX_AT_LEAST_KINETIC
    auto &world = scene_->getWorldNonConst();
    success = world->moveObject(name, transform);
#endif
    if (not success)
        ROS_ERROR("Failed to move object %s", name.c_str());

    return success;
}

bool Scene::moveObjectLocal(const std::string &name, const RobotPose &transform)
{
    incrementVersion();

    const auto pose = getObjectPose(name);
    const auto global_tf = pose * transform * pose.inverse();

    bool success = moveObjectGlobal(name, global_tf);
    return success;
}

RobotPose Scene::getFramePose(const std::string &id) const
{
    if (not scene_->knowsFrameTransform(id))
        ROS_WARN("Frame %s in not present in the scene!", id.c_str());

    return scene_->getFrameTransform(id);
}

bool Scene::setCollisionDetector(const std::string &detector_name) const
{
    bool success = true;
    if (not loader_->activate(detector_name, scene_, true))
    {
        success = false;
        ROS_WARN("Was not able to load collision detector plugin '%s'", detector_name.c_str());
    }

    ROS_INFO("Using collision detector: %s", scene_->getActiveCollisionDetectorName().c_str());
    return success;
}

bool Scene::attachObject(const std::string &name)
{
    const auto &robot = getCurrentState().getRobotModel();
    const auto &ee = robot->getEndEffectors();

    // One end-effector
    if (ee.size() == 1)
    {
        const auto &links = ee[0]->getLinkModelNames();
        return attachObject(name, links[0], links);
    }

    return false;
}

bool Scene::attachObject(robot_state::RobotState &state, const std::string &name)
{
    const auto &robot = state.getRobotModel();
    const auto &ee = robot->getEndEffectors();

    // One end-effector
    if (ee.size() == 1)
    {
        const auto &links = ee[0]->getLinkModelNames();
        return attachObject(state, name, links[0], links);
    }

    return false;
}

bool Scene::attachObject(const std::string &name, const std::string &ee_link,
                         const std::vector<std::string> &touch_links)
{
    incrementVersion();

    auto &world = scene_->getWorldNonConst();
    if (!world->hasObject(name))
    {
        ROS_ERROR("World does not have object `%s`", name.c_str());
        return false;
    }

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

    auto &scene_state = getCurrentState();
    scene_state.attachBody(name, obj->shapes_, obj->shape_poses_, touch_links, ee_link);
    return true;
}

bool Scene::attachObject(robot_state::RobotState &state, const std::string &name, const std::string &ee_link,
                         const std::vector<std::string> &touch_links)
{
    incrementVersion();

    auto &world = scene_->getWorldNonConst();
    if (!world->hasObject(name))
    {
        ROS_ERROR("World does not have object `%s`", name.c_str());
        return false;
    }

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

    scene_->setCurrentState(state);
    const auto &tf = state.getGlobalLinkTransform(ee_link);

    RobotPoseVector poses;
    for (const auto &pose : obj->shape_poses_)
        poses.push_back(tf.inverse() * pose);

    auto &scene_state = getCurrentState();
    scene_state.attachBody(name, obj->shapes_, poses, touch_links, ee_link);
    return true;
}

bool Scene::hasObject(const std::string &name) const
{
    const auto &world = scene_->getWorld();
    return world->hasObject(name);
}

bool Scene::detachObject(const std::string &name)
{
    incrementVersion();

    auto &robot = scene_->getCurrentStateNonConst();
    auto &world = scene_->getWorldNonConst();
    auto body = robot.getAttachedBody(name);

    if (!body)
    {
        ROS_ERROR("Robot does not have attached object `%s`", name.c_str());
        return false;
    }

    world->addToObject(name, body->getShapes(), body->getFixedTransforms());

    if (!robot.clearAttachedBody(name))
    {
        ROS_ERROR("Could not detach object `%s`", name.c_str());
        return false;
    }

    return true;
}

collision_detection::CollisionResult Scene::checkCollision(
    const robot_state::RobotState &state, const collision_detection::CollisionRequest &request) const
{
    collision_detection::CollisionResult result;
    scene_->checkCollision(request, result, state);

    return result;
}

double Scene::distanceToCollision(const robot_state::RobotStatePtr &state) const
{
    return scene_->distanceToCollision(*state);
}

double Scene::distanceToObject(const robot_state::RobotStatePtr &state, const std::string &object) const
{
#if ROBOWFLEX_AT_LEAST_KINETIC and ROBOWFLEX_AT_MOST_MELODIC
    if (not hasObject(object))
    {
        ROS_ERROR("World does not have object `%s`", object.c_str());
        return std::numeric_limits<double>::quiet_NaN();
    }

    const auto &cw = scene_->getCollisionWorld();
    const auto &cr = *scene_->getCollisionRobot();

    collision_detection::DistanceRequest req;
    collision_detection::DistanceResult res;

    const auto &links = state->getRobotModel()->getLinkModelNames();
    const auto &objs = getCollisionObjects();

    collision_detection::AllowedCollisionMatrix acm;

    // No self-collision distances
    for (unsigned int i = 0; i < links.size(); ++i)
        for (unsigned int j = i + 1; j < links.size(); ++j)
            acm.setEntry(links[i], links[j], true);

    // Ignore all other objects
    for (unsigned int i = 0; i < links.size(); ++i)
        for (unsigned int j = 0; j < objs.size(); ++j)
            acm.setEntry(links[i], objs[j], true);

    // Enable collision to the object of interest
    for (unsigned int i = 0; i < links.size(); ++i)
        acm.setEntry(links[i], object, false);

    req.acm = &acm;

    cw->distanceRobot(req, res, cr, *state);
    return res.minimum_distance.distance;

#else
    throw std::runtime_error("Not Implemented");

#endif
}

double Scene::distanceBetweenObjects(const std::string &one, const std::string &two) const
{
#if ROBOWFLEX_AT_LEAST_KINETIC and ROBOWFLEX_AT_MOST_MELODIC
    // Early terminate if they are the same
    if (one == two)
        return 0.;

    if (not hasObject(one))
    {
        ROS_ERROR("World does not have object `%s`", one.c_str());
        return std::numeric_limits<double>::quiet_NaN();
    }

    if (not hasObject(two))
    {
        ROS_ERROR("World does not have object `%s`", two.c_str());
        return std::numeric_limits<double>::quiet_NaN();
    }

    const auto &cw = scene_->getCollisionWorld();

    collision_detection::DistanceRequest req;
    collision_detection::DistanceResult res;

    const auto &objs = getCollisionObjects();

    // Allow collisions between all other objects
    collision_detection::AllowedCollisionMatrix acm(objs, true);
    req.acm = &acm;

    // But disable them for the two we care about
    acm.setEntry(one, two, false);

    cw->distanceWorld(req, res, *cw);
    return res.minimum_distance.distance;

#else
    throw std::runtime_error("Not Implemented");

#endif
}

bool Scene::toYAMLFile(const std::string &file) const
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);

    YAML::Node node = IO::toNode(msg);
    return IO::YAMLToFile(node, file);
}

bool Scene::fromYAMLFile(const std::string &file)
{
    moveit_msgs::PlanningScene msg;
    if (!IO::fromYAMLFile(msg, file))
        return false;

    auto acm(getACM());
    useMessage(msg);

    // Update ACM only if anything specified.
    if (msg.allowed_collision_matrix.entry_names.empty())
        getACM() = acm;

    return true;
}

bool Scene::fromOpenRAVEXMLFile(const std::string &file, std::string models_dir)
{
    if (models_dir.empty())
        models_dir = IO::resolveParent(file);

    moveit_msgs::PlanningScene msg;
    if (!openrave::fromXMLFile(msg, file, models_dir))
        return false;

    scene_->usePlanningSceneMsg(msg);
    return true;
}
