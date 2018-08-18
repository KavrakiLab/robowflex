/* Author: Zachary Kingston */

#include <deque>
#include <numeric>

#include <urdf_parser/urdf_parser.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_common.h>

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
    bool success =
        loadXMLFile(ROBOT_DESCRIPTION, urdf_file, urdf_function_)                           // urdf
        && loadXMLFile(ROBOT_DESCRIPTION + ROBOT_SEMANTIC, srdf_file, srdf_function_)       // srdf
        && loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_PLANNING, limits_file, limits_function_)  // joint limits
        && loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_KINEMATICS, kinematics_file, kinematics_function_);

    return success;
}

void Robot::setURDFPostProcessFunction(const PostProcessXMLFunction &function)
{
    urdf_function_ = function;
}

void Robot::setSRDFPostProcessFunction(const PostProcessXMLFunction &function)
{
    srdf_function_ = function;
}

void Robot::setLimitsPostProcessFunction(const PostProcessYAMLFunction &function)
{
    limits_function_ = function;
}

void Robot::setKinematicsPostProcessFunction(const PostProcessYAMLFunction &function)
{
    kinematics_function_ = function;
}

bool Robot::loadYAMLFile(const std::string &name, const std::string &file)
{
    PostProcessYAMLFunction function;
    return loadYAMLFile(name, file, function);
}

bool Robot::loadYAMLFile(const std::string &name, const std::string &file,
                         const PostProcessYAMLFunction &function)
{
    auto &yaml = IO::loadFileToYAML(file);
    if (!yaml.first)
    {
        ROS_ERROR("Failed to load YAML file `%s`.", file.c_str());
        return false;
    }

    if (function)
    {
        YAML::Node copy = yaml.second;
        if (!function(copy))
        {
            ROS_ERROR("Failed to process YAML file `%s`.", file.c_str());
            return false;
        }

        handler_.loadYAMLtoROS(copy, name);
    }
    else
        handler_.loadYAMLtoROS(yaml.second, name);

    return true;
}

bool Robot::loadXMLFile(const std::string &name, const std::string &file)
{
    PostProcessXMLFunction function;
    return loadXMLFile(name, file, function);
}

bool Robot::loadXMLFile(const std::string &name, const std::string &file,
                        const PostProcessXMLFunction &function)
{
    const std::string string = IO::loadXMLToString(file);
    if (string.empty())
    {
        ROS_ERROR("Failed to load XML file `%s`.", file.c_str());
        return false;
    }

    if (function)
    {
        tinyxml2::XMLDocument doc;
        doc.Parse(string.c_str());

        if (!function(doc))
        {
            ROS_ERROR("Failed to process XML file `%s`.", file.c_str());
            return false;
        }

        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);

        handler_.setParam(name, std::string(printer.CStr()));
    }
    else
        handler_.setParam(name, string);

    return true;
}

void Robot::loadRobotModel(bool namespaced)
{
    robot_model_loader::RobotModelLoader::Options options(((namespaced) ? handler_.getNamespace() : "") +
                                                          "/" + ROBOT_DESCRIPTION);
    options.load_kinematics_solvers_ = false;

    loader_.reset(new robot_model_loader::RobotModelLoader(options));
    kinematics_.reset(new kinematics_plugin_loader::KinematicsPluginLoader(loader_->getRobotDescription()));

    model_ = loader_->getModel();
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

urdf::ModelInterfaceConstSharedPtr Robot::getURDF() const
{
    return model_->getURDF();
}

srdf::ModelConstSharedPtr Robot::getSRDF() const
{
    return model_->getSRDF();
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

void Robot::setState(const moveit_msgs::RobotState &state)
{
    moveit::core::robotStateMsgToRobotState(state, *scratch_);
    scratch_->update();
}

void Robot::setStateFromYAMLFile(const std::string &file)
{
    moveit_msgs::RobotState state;
    IO::fromYAMLFile(state, file);

    setState(state);
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

bool Robot::setFromIK(const std::string &group,                                     //
                      const GeometryConstPtr &region, const Eigen::Affine3d &pose,  //
                      const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances)
{
    Eigen::Affine3d sampled_pose = pose;
    auto sample = region->sample();
    if (!sample.first)
        return false;

    sampled_pose.translate(sample.second);
    sampled_pose.rotate(TF::sampleOrientation(orientation, tolerances));

    geometry_msgs::Pose msg = TF::poseEigenToMsg(sampled_pose);
    robot_model::JointModelGroup *jmg = model_->getJointModelGroup(group);

    if (scratch_->setFromIK(jmg, msg))
    {
        scratch_->update();
        return true;
    }

    return false;
}

const Eigen::Affine3d &Robot::getLinkTF(const std::string &name) const
{
    return scratch_->getGlobalLinkTransform(name);
}

const Eigen::Affine3d Robot::getRelativeLinkTF(const std::string &base, const std::string &target) const
{
    auto base_tf = scratch_->getGlobalLinkTransform(base);
    auto target_tf = scratch_->getGlobalLinkTransform(target);

    return base_tf.inverse() * target_tf;
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

    void addLinkOrigin(YAML::Node &node, const urdf::Pose &pose)
    {
        YAML::Node origin;
        origin["position"] = std::vector<double>({pose.position.x, pose.position.y, pose.position.z});
        origin["orientation"] = std::vector<double>({pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w});
        node["origin"] = origin;
        ROBOWFLEX_YAML_FLOW(node["origin"]);
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

                const auto &pose = visual->origin;

                // TODO: Also check if rotation is not zero.
                if (pose.position.x != 0 || pose.position.y != 0 || pose.position.z != 0)
                    addLinkOrigin(node, pose);
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

    Eigen::Affine3d urdfPoseToEigen(const urdf::Pose &pose)
    {
        geometry_msgs::Pose msg;
        msg.position.x = pose.position.x;
        msg.position.y = pose.position.y;
        msg.position.z = pose.position.z;

        msg.orientation.x = pose.rotation.x;
        msg.orientation.y = pose.rotation.y;
        msg.orientation.z = pose.rotation.z;
        msg.orientation.w = pose.rotation.w;

        return TF::poseMsgToEigen(msg);
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

        YAML::Node visual;
#if ROBOWFLEX_AT_LEAST_KINETIC
        for (const auto &element : link->visual_array)
            if (element)
                visual["elements"].push_back(addLinkVisual(element));
#else
        if (link->visual)
            visual["elements"].push_back(addLinkVisual(link->visual));
#endif

        YAML::Node collision;
#if ROBOWFLEX_AT_LEAST_KINETIC
        for (const auto &element : link->collision_array)
            if (element)
                collision["elements"].push_back(addLinkCollision(element));
#else
        if (link->collision)
            collision["elements"].push_back(addLinkCollision(link->collision));
#endif

        if (!visual.IsNull() || !collision.IsNull())
        {
            YAML::Node add;
            add["name"] = link->name;

            if (!visual.IsNull())
                add["visual"] = visual;

            if (!collision.IsNull())
                add["collision"] = collision;

            link_geometry.push_back(add);
        }
    }

    return IO::YAMLToFile(link_geometry, filename);
}

bool Robot::dumpTransforms(const std::string &filename) const
{
    robot_trajectory::RobotTrajectory trajectory(model_, "");
    trajectory.addSuffixWayPoint(scratch_, 0.0);

    return dumpPathTransforms(trajectory, filename, 0., 0.);
}

bool Robot::dumpPathTransforms(const robot_trajectory::RobotTrajectory &path, const std::string &filename,
                               double fps, double threshold) const
{
    YAML::Node node, values;
    const double rate = 1.0 / fps;

    // Find the total duration of the path.
    const std::deque<double> &durations = path.getWayPointDurations();
    double total_duration = std::accumulate(durations.begin(), durations.end(), 0.0);

    const auto &urdf = model_->getURDF();

    robot_state::RobotStatePtr previous, state(new robot_state::RobotState(model_));

    for (double duration = 0.0, delay = 0.0; duration <= total_duration; duration += rate, delay += rate)
    {
        YAML::Node point;

        path.getStateAtDurationFromStart(duration, state);
        if (previous && state->distance(*previous) < threshold)
            continue;

        state->update();
        for (const auto &link_name : model_->getLinkModelNames())
        {
            const auto &urdf_link = urdf->getLink(link_name);
            if (urdf_link->visual)
            {
                const auto &link = model_->getLinkModel(link_name);
                Eigen::Affine3d tf =
                    state->getGlobalLinkTransform(link) * urdfPoseToEigen(urdf_link->visual->origin);
                point[link->getName()] = IO::toNode(TF::poseEigenToMsg(tf));
            }
        }

        YAML::Node value;
        value["point"] = point;
        value["duration"] = delay;
        values.push_back(value);

        delay = 0;

        if (!previous)
            previous.reset(new robot_state::RobotState(model_));

        *previous = *state;
    }

    node["transforms"] = values;
    node["fps"] = fps;

    return IO::YAMLToFile(node, filename);
}

///
/// robowflex::ParamRobot
///

ParamRobot::ParamRobot(const std::string &name) : Robot(name)
{
    loadRobotModel(false);
}
