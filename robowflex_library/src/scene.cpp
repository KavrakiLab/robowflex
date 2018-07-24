/* Author: Zachary Kingston */

#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>

#include <pluginlib/class_loader.h>
#include <moveit/collision_detection/collision_plugin.h>

namespace robowflex
{
    /** \brief The actual plugin loader for collision plugins.
     *  Heavily inspired by code in moveit_ros/planning. */
    class Scene::CollisionPluginLoaderImpl
    {
    public:
        CollisionPluginLoaderImpl()
        {
            try
            {
                loader_.reset(new pluginlib::ClassLoader<collision_detection::CollisionPlugin>("moveit_core", "collision_detection::CollisionPlugin"));
            }
            catch (pluginlib::PluginlibException &e)
            {
                ROS_ERROR("Unable to construct collision plugin loader. Error: %s", e.what());
            }
        }

        /** \brief Attempts to load the collision detector plugin by the given \a name.
         *  Saves the plugin in an internal map and returns it if found.
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
         *  \param[it] exclusive If true, the new collision detector is the only one.
         *  \return True if the new collision detector is added to the scene.
         */
        bool activate(const std::string& name, const planning_scene::PlanningScenePtr& scene, bool exclusive)
        {
            auto it = plugins_.find(name);
            if (it == plugins_.end())
            {
                collision_detection::CollisionPluginPtr plugin = load(name);
                if (plugin)
                {
                    return plugin->initialize(scene, exclusive);
                }
                return false;
            }
            if (it->second)
            {
                return it->second->initialize(scene, exclusive);
            }
        }

    private:
        std::shared_ptr<pluginlib::ClassLoader<collision_detection::CollisionPlugin> > loader_;
        std::map<std::string, collision_detection::CollisionPluginPtr> plugins_; ///< A map of already loaded plugins.
    };
}



using namespace robowflex;

Scene::Scene(const RobotConstPtr &robot) : scene_(new planning_scene::PlanningScene(robot->getModelConst()))
{
    plugin_loader_.reset(new CollisionPluginLoaderImpl());
}

Scene::Scene(const Scene &other) : scene_(other.getSceneConst())
{
    plugin_loader_.reset(new CollisionPluginLoaderImpl());
}

void Scene::operator=(const Scene &other)
{
    scene_ = other.getSceneConst();
}

const planning_scene::PlanningScenePtr &Scene::getSceneConst() const
{
    return scene_;
}

planning_scene::PlanningScenePtr &Scene::getScene()
{
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
    return scene_->getCurrentStateNonConst();
}

collision_detection::AllowedCollisionMatrix &Scene::getACM()
{
    return scene_->getAllowedCollisionMatrixNonConst();
}

void Scene::updateCollisionObject(const std::string &name, const GeometryConstPtr &geometry,
                                  const Eigen::Affine3d &pose)
{
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

Eigen::Affine3d Scene::getFramePose(const std::string &id) const
{
    if (not scene_->knowsFrameTransform(id))
    {
        ROS_WARN("Frame %s in not present in the scene!", id.c_str());
    }
    return scene_->getFrameTransform(id);
}

bool Scene::setCollisionDetector(const std::string &detector_name) const
{
    bool success = true;
    if (not plugin_loader_->activate(detector_name, scene_, true))
    {
        success = false;
        ROS_WARN("Was not able to load collision detector plugin '%s'", detector_name.c_str());
    }
    ROS_INFO("Using collision detector: %s", scene_->getActiveCollisionDetectorName().c_str());
    return success;
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
    return true;
}

bool Scene::toYAMLFile(const std::string &file)
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);

    YAML::Node node = IO::toNode(msg);
    return IO::YAMLtoFile(node, file);
}

bool Scene::fromYAMLFile(const std::string &file)
{
    moveit_msgs::PlanningScene msg;
    if (!IO::fromYAMLFile(msg, file))
        return false;

    auto acm(getACM());
    scene_->setPlanningSceneMsg(msg);

    // Update ACM only if anything specified.
    if (msg.allowed_collision_matrix.entry_names.empty())
        getACM() = acm;

    return true;
}
