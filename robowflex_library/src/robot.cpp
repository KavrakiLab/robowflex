/* Author: Zachary Kingston */

#include <deque>
#include <numeric>

#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>

#include <robowflex_library/macros.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

using namespace robowflex;

const std::string Robot::ROBOT_DESCRIPTION = "robot_description";
const std::string Robot::ROBOT_SEMANTIC = "_semantic";
const std::string Robot::ROBOT_PLANNING = "_planning";
const std::string Robot::ROBOT_KINEMATICS = "_kinematics";

Robot::Robot(const std::string &name) : name_(name), handler_(name_)
{
}

bool Robot::initialize(const std::string &urdf_file, const std::string &srdf_file,
                       const std::string &limits_file, const std::string &kinematics_file)
{
    if (!loadRobotDescription(urdf_file, srdf_file, limits_file, kinematics_file))
        return false;

    loadRobotModel();
    return true;
}

bool Robot::loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                 const std::string &limits_file, const std::string &kinematics_file)
{
    bool success = loadXMLFile(ROBOT_DESCRIPTION, urdf_file)                                // urdf
                   && loadXMLFile(ROBOT_DESCRIPTION + ROBOT_SEMANTIC, srdf_file)            // srdf
                   && loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_PLANNING, limits_file)         // joint limits
                   && loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_KINEMATICS, kinematics_file);  // kinematics

    return success;
}

bool Robot::loadYAMLFile(const std::string &name, const std::string &file)
{
    if (!handler_.hasParam(name))
    {
        auto &yaml = IO::loadFileToYAML(file);
        if (!yaml.first)
        {
            ROS_ERROR("Failed to load YAML file `%s`.", file.c_str());
            return false;
        }

        handler_.loadYAMLtoROS(yaml.second, name);
    }

    return true;
}

bool Robot::loadXMLFile(const std::string &name, const std::string &file)
{
    if (!handler_.hasParam(name))
    {
        const std::string string = IO::loadXMLToString(file);
        if (string.empty())
        {
            ROS_ERROR("Failed to load XML file `%s`.", file.c_str());
            return false;
        }

        handler_.setParam(name, string);
    }

    return true;
}

void Robot::loadRobotModel()
{
    robot_model_loader::RobotModelLoader::Options options(handler_.getNamespace() + "/" + ROBOT_DESCRIPTION);
    options.load_kinematics_solvers_ = false;

    loader_.reset(new robot_model_loader::RobotModelLoader(options));
    kinematics_.reset(new kinematics_plugin_loader::KinematicsPluginLoader(loader_->getRobotDescription()));

    model_ = std::move(loader_->getModel());
    scratch_.reset(new robot_state::RobotState(model_));
}

bool Robot::loadKinematics(const std::string &name)
{
    if (imap_.find(name) != imap_.end())
        return true;

    robot_model::SolverAllocatorFn allocator = kinematics_->getLoaderFunction(loader_->getSRDF());

    const auto &groups = kinematics_->getKnownGroups();
    if (groups.empty())
    {
        ROS_WARN("No kinematics plugins defined. Fill and load kinematics.yaml!");
        return false;
    }

    if (!model_->hasJointModelGroup(name) || std::find(groups.begin(), groups.end(), name) == groups.end())
    {
        ROS_WARN("No JMG or Kinematics defined for `%s`!", name.c_str());
        return false;
    }

    robot_model::JointModelGroup *jmg = model_->getJointModelGroup(name);
    kinematics::KinematicsBasePtr solver = allocator(jmg);

    if (solver)
    {
        std::string error_msg;
        if (solver->supportsGroup(jmg, &error_msg))
            imap_[name] = allocator;

        else
        {
            ROS_ERROR("Kinematics solver %s does not support joint group %s.  Error: %s",
                      typeid(*solver).name(), name.c_str(), error_msg.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Kinematics solver could not be instantiated for joint group `%s`.", name.c_str());
        return false;
    }

    auto timeout = kinematics_->getIKTimeout();
    auto attempts = kinematics_->getIKAttempts();

    jmg->setDefaultIKTimeout(timeout[name]);
    jmg->setDefaultIKAttempts(attempts[name]);

    model_->setKinematicsAllocators(imap_);

    return true;
}

const std::string &Robot::getModelName() const
{
    return model_->getName();
}

const std::string &Robot::getName() const
{
    return name_;
}

const robot_model::RobotModelPtr &Robot::getModelConst() const
{
    return model_;
}

robot_model::RobotModelPtr &Robot::getModel()
{
    return model_;
}

const robot_model::RobotStatePtr &Robot::getScratchState() const
{
    return scratch_;
}

robot_model::RobotStatePtr &Robot::getScratchState()
{
    return scratch_;
}

const IO::Handler &Robot::getHandlerConst() const
{
    return handler_;
}

IO::Handler &Robot::getHandler()
{
    return handler_;
}

void Robot::setState(const std::vector<double> &positions)
{
    scratch_->setVariablePositions(positions);
    scratch_->update();
}

void Robot::setState(const std::map<std::string, double> &variable_map)
{
    scratch_->setVariablePositions(variable_map);
    scratch_->update();
}

void Robot::setState(const std::vector<std::string> &variable_names,
                     const std::vector<double> &variable_position)
{
    scratch_->setVariablePositions(variable_names, variable_position);
    scratch_->update();
}

void Robot::setGroupState(const std::string &name, const std::vector<double> &positions)
{
    scratch_->setJointGroupPositions(name, positions);
    scratch_->update();
}

std::vector<double> Robot::getState() const
{
    const double *positions = scratch_->getVariablePositions();
    std::vector<double> state(positions, positions + scratch_->getVariableCount());
    return state;
}

std::vector<std::string> Robot::getJointNames() const
{
    return scratch_->getVariableNames();
}

void Robot::setFromIK(const std::string &group,                                     //
                      const GeometryConstPtr &region, const Eigen::Affine3d &pose,  //
                      const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances)
{
    Eigen::Affine3d sampled_pose = pose;

    sampled_pose.translate(region->sample());
    sampled_pose.rotate(TF::sampleOrientation(orientation, tolerances));

    geometry_msgs::Pose msg = TF::poseEigenToMsg(sampled_pose);

    robot_model::JointModelGroup *jmg = model_->getJointModelGroup(group);
    scratch_->setFromIK(jmg, msg);
    scratch_->update();
}

const Eigen::Affine3d &Robot::getLinkTF(const std::string &name) const
{
    return scratch_->getGlobalLinkTransform(name);
}

bool Robot::inCollision(const SceneConstPtr &scene) const
{
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    scene->getSceneConst()->checkCollisionUnpadded(request, result, *scratch_);

    return result.collision;
}

namespace
{
    YAML::Node addLinkGeometry(const urdf::GeometrySharedPtr &geometry)
    {
        YAML::Node node;
        switch (geometry->type)
        {
            case urdf::Geometry::MESH:
            {
                const auto &mesh = static_cast<urdf::Mesh *>(geometry.get());
                node["type"] = "mesh";
                node["resource"] = IO::resolvePath(mesh->filename);

                if (mesh->scale.x != 1 || mesh->scale.y != 1 || mesh->scale.z != 1)
                {
                    node["dimensions"] = std::vector<double>({mesh->scale.x, mesh->scale.y, mesh->scale.z});
                    ROBOWFLEX_YAML_FLOW(node["dimensions"]);
                }
                break;
            }
            case urdf::Geometry::BOX:
            {
                const auto &box = static_cast<urdf::Box *>(geometry.get());
                node["type"] = "box";
                node["dimensions"] = std::vector<double>({box->dim.x, box->dim.y, box->dim.z});
                ROBOWFLEX_YAML_FLOW(node["dimensions"]);
                break;
            }
            case urdf::Geometry::SPHERE:
            {
                const auto &sphere = static_cast<urdf::Sphere *>(geometry.get());
                node["type"] = "sphere";
                node["dimensions"] = std::vector<double>({sphere->radius});
                ROBOWFLEX_YAML_FLOW(node["dimensions"]);
                break;
            }
            case urdf::Geometry::CYLINDER:
            {
                const auto &cylinder = static_cast<urdf::Cylinder *>(geometry.get());
                node["type"] = "cylinder";
                node["dimensions"] = std::vector<double>({cylinder->length, cylinder->radius});
                ROBOWFLEX_YAML_FLOW(node["dimensions"]);
                break;
            }
            default:
                break;
        }

        return node;
    }

    void addLinkMaterial(YAML::Node &node, const urdf::MaterialSharedPtr &material)
    {
        node["color"] =
            std::vector<double>({material->color.r, material->color.g, material->color.b, material->color.a});

        ROBOWFLEX_YAML_FLOW(node["color"]);
        // node["texture"] = visual->texture_filename;
    }

    YAML::Node addLinkVisual(const urdf::VisualSharedPtr &visual)
    {
        YAML::Node node;
        if (visual)
        {
            if (visual->geometry)
            {
                const auto &geometry = visual->geometry;
                node = addLinkGeometry(geometry);

                if (visual->material)
                {
                    const auto &material = visual->material;
                    addLinkMaterial(node, material);
                }
            }
        }

        return node;
    }

    YAML::Node addLinkCollision(const urdf::CollisionSharedPtr &collision)
    {
        YAML::Node node;
        if (collision)
        {
            if (collision->geometry)
            {
                const auto &geometry = collision->geometry;
                node = addLinkGeometry(geometry);
            }
        }

        return node;
    }
}  // namespace

bool Robot::dumpGeometry(const std::string &filename) const
{
    YAML::Node link_geometry;
    const auto &urdf = model_->getURDF();

    std::vector<urdf::LinkSharedPtr> links;
    urdf->getLinks(links);

    for (const auto &link : links)
    {
        YAML::Node node;

        bool has_visual = false;
        YAML::Node visual;
        if (link->visual)
        {
            visual["visual"] = addLinkVisual(link->visual);
            has_visual = true;
        }

        if (!link->visual_groups.empty())
        {
            YAML::Node visual_groups;
            for (const auto &group_pair : link->visual_groups)
            {
                YAML::Node geometry, group;
                for (const auto &visual : *group_pair.second)
                    if (visual)
                        geometry.push_back(addLinkVisual(visual));

                group["name"] = group_pair.first;
                group["elements"] = geometry;
                visual_groups.push_back(group);
            }

            visual["groups"] = visual_groups;
            has_visual = true;
        }

        if (has_visual)
            node["visual"] = visual;

        bool has_collision = false;
        YAML::Node collision;
        if (link->collision)
        {
            collision["collision"] = addLinkCollision(link->collision);
            has_collision = true;
        }

        if (!link->collision_groups.empty())
        {
            YAML::Node collision_groups;
            for (const auto &group_pair : link->collision_groups)
            {
                YAML::Node geometry, group;
                for (const auto &collision : *group_pair.second)
                    if (collision)
                        geometry.push_back(addLinkCollision(collision));

                group["name"] = group_pair.first;
                group["elements"] = geometry;
                collision_groups.push_back(group);
            }

            collision["groups"] = collision_groups;
            has_collision = true;
        }

        if (has_collision)
            node["collision"] = collision;

        if (has_visual || has_collision)
        {
            YAML::Node add;
            add["name"] = link->name;
            add["geometry"] = node;
            link_geometry.push_back(add);
        }
    }

    return IO::YAMLtoFile(link_geometry, filename);
}

bool Robot::dumpPathTransforms(const robot_trajectory::RobotTrajectory &path, const std::string &filename,
                               double fps)
{
    YAML::Node node, values;

    // Find the total duration of the path.
    const std::deque<double> &durations = path.getWayPointDurations();
    double total_duration = std::accumulate(durations.begin(), durations.end(), 0.0);

    robot_state::RobotStatePtr state(new robot_state::RobotState(model_));
    for (double duration = 0.0; duration < total_duration; duration += (1.0 / fps))
    {
        YAML::Node point;

        path.getStateAtDurationFromStart(duration, state);
        state->update();

        for (const auto &link : model_->getLinkModels())
        {
            Eigen::Affine3d tf = state->getGlobalLinkTransform(link) * link->getVisualMeshOrigin();
            point[link->getName()] = IO::toNode(TF::poseEigenToMsg(tf));
        }

        YAML::Node value;
        value["point"] = point;
        value["duration"] = 1.0 / fps;
        values.push_back(value);
        break;
    }

    node["fps"] = fps;
    node["transforms"] = values;

    return IO::YAMLtoFile(node, filename);
}
