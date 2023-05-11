/* Author: Zachary Kingston */

#include <deque>
#include <numeric>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/log.h>
#include <robowflex_library/macros.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>

using namespace robowflex;

const std::string Robot::ROBOT_DESCRIPTION = "robot_description";
const std::string Robot::ROBOT_SEMANTIC = "_semantic";
const std::string Robot::ROBOT_PLANNING = "_planning";
const std::string Robot::ROBOT_KINEMATICS = "_kinematics";

Robot::Robot(const std::string &name) : name_(name), handler_(name_)
{
}

bool Robot::loadURDFFile(const std::string &urdf_file)
{
    std::string urdf = loadXMLFile(urdf_file);
    if (urdf.empty())
        return false;

    urdf_ = urdf;
    return true;
}

bool Robot::loadSRDFFile(const std::string &srdf_file)
{
    std::string srdf = loadXMLFile(srdf_file);
    if (srdf.empty())
        return false;

    srdf_ = srdf;
    return true;
}

bool Robot::initialize(const std::string &urdf_file, const std::string &srdf_file,
                       const std::string &limits_file, const std::string &kinematics_file)
{
    if (loader_)
    {
        RBX_ERROR("Already initialized!");
        return false;
    }

    if (not loadURDFFile(urdf_file))
    {
        RBX_ERROR("Failed to load URDF!");
        return false;
    }

    if (not loadSRDFFile(srdf_file))
    {
        RBX_ERROR("Failed to load SRDF!");
        return false;
    }

    if (not limits_file.empty())
        if (not loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_PLANNING, limits_file, limits_function_))
        {
            RBX_ERROR("Failed to load joint limits!");
            return false;
        }

    if (not kinematics_file.empty())
        if (not initializeKinematics(kinematics_file))
        {
            RBX_ERROR("Failed to load kinematics!");
            return false;
        }

    initializeInternal();
    return true;
}

bool Robot::initializeFromYAML(const std::string &config_file)
{
    if (loader_)
    {
        RBX_ERROR("Already initialized!");
        return false;
    }

    const auto &yaml = IO::loadFileToYAML(config_file);
    if (not yaml.first)
    {
        RBX_ERROR("Failed to load YAML file `%s`.", config_file);
        return false;
    }

    const auto &node = yaml.second;

    // URDF
    std::string urdf_file;
    if (IO::isNode(node["urdf"]))
        urdf_file = node["urdf"].as<std::string>();
    else
    {
        RBX_ERROR("No URDF entry in YAML file `%s`!", config_file);
        return false;
    }

    // SRDF
    std::string srdf_file;
    if (IO::isNode(node["srdf"]))
        srdf_file = node["srdf"].as<std::string>();
    else
        RBX_WARN("No SRDF entry in YAML!");

    // Joint limits
    std::string limits_file;
    if (IO::isNode(node["limits"]))
    {
        if (srdf_file.empty())
        {
            RBX_ERROR("Cannot provide joint limits without SRDF in YAML file `%s`!", config_file);
            return false;
        }

        limits_file = node["limits"].as<std::string>();
    }
    else
        RBX_WARN("No joint limits provided!");

    // Kinematics plugins
    std::string kinematics_file;
    if (IO::isNode(node["kinematics"]))
    {
        if (srdf_file.empty())
        {
            RBX_ERROR("Cannot provide kinematics without SRDF in YAML file `%s`!", config_file);
            return false;
        }

        kinematics_file = node["kinematics"].as<std::string>();
    }
    else
        RBX_WARN("No kinematics plugins provided!");

    // Initialize robot
    bool r;
    if (srdf_file.empty())
        r = initialize(urdf_file);
    else
        r = initialize(urdf_file, srdf_file, limits_file, kinematics_file);

    // Set default state if provided in file.
    if (r)
    {
        if (IO::isNode(node["robot_state"]))
        {
            const auto &robot_state = IO::robotStateFromNode(node["robot_state"]);
            setState(robot_state);
        }
        else
            RBX_WARN("No default state provided!");
    }

    return r;
}

bool Robot::initialize(const std::string &urdf_file)
{
    if (loader_)
    {
        RBX_ERROR("Already initialized!");
        return false;
    }

    if (not loadURDFFile(urdf_file))
        return false;

    // Generate basic SRDF on the fly.
    tinyxml2::XMLDocument doc;
    doc.Parse(urdf_.c_str());
    const auto &name = doc.FirstChildElement("robot")->Attribute("name");
    const std::string &srdf = "<?xml version=\"1.0\" ?><robot name=\"" + std::string(name) + "\"></robot>";

    srdf_ = srdf;

    initializeInternal();

    return true;
}

bool Robot::initializeKinematics(const std::string &kinematics_file)
{
    if (kinematics_)
    {
        RBX_ERROR("Already loaded kinematics!");
        return false;
    }

    return loadYAMLFile(ROBOT_DESCRIPTION + ROBOT_KINEMATICS, kinematics_file, kinematics_function_);
}

void Robot::setURDFPostProcessFunction(const PostProcessXMLFunction &function)
{
    urdf_function_ = function;
}

bool Robot::isLinkURDF(tinyxml2::XMLDocument &doc, const std::string &name)
{
    auto *node = doc.FirstChildElement("robot")->FirstChildElement("link");
    while (node != nullptr)
    {
        if (node->Attribute("name", name.c_str()))
            return true;

        node = node->NextSiblingElement("link");
    }
    return false;
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
    const auto &yaml = IO::loadFileToYAML(file);
    if (!yaml.first)
    {
        RBX_ERROR("Failed to load YAML file `%s`.", file);
        return false;
    }

    if (function)
    {
        YAML::Node copy = yaml.second;
        if (!function(copy))
        {
            RBX_ERROR("Failed to process YAML file `%s`.", file);
            return false;
        }

        handler_.loadYAMLtoROS(copy, name);
    }
    else
        handler_.loadYAMLtoROS(yaml.second, name);

    return true;
}

std::string Robot::loadXMLFile(const std::string &file)
{
    std::string string = IO::loadXMLToString(file);
    if (string.empty())
    {
        RBX_ERROR("Failed to load XML file `%s`.", file);
        return "";
    }

    return string;
}

void Robot::updateXMLString(std::string &string, const PostProcessXMLFunction &function)
{
    if (function)
    {
        tinyxml2::XMLDocument doc;
        doc.Parse(string.c_str());

        if (not function(doc))
        {
            RBX_ERROR("Failed to process XML string `%s`.", string);
            return;
        }

        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);

        string = std::string(printer.CStr());
    }
}

void Robot::initializeInternal(bool namespaced)
{
    const std::string &description = ((namespaced) ? handler_.getNamespace() : "") + "/" + ROBOT_DESCRIPTION;

    loadRobotModel(description);
    if (urdf_function_)
        updateXMLString(urdf_, urdf_function_);

    if (srdf_function_)
        updateXMLString(srdf_, srdf_function_);

    // If either function was called, reload robot.
    if (urdf_function_ or srdf_function_)
    {
        RBX_INFO("Reloading model after URDF/SRDF post-process function...");
        loadRobotModel(description);
    }

    // set strings on parameter server
    handler_.setParam(ROBOT_DESCRIPTION, urdf_);
    handler_.setParam(ROBOT_DESCRIPTION + ROBOT_SEMANTIC, srdf_);

    scratch_.reset(new robot_state::RobotState(model_));
    scratch_->setToDefaultValues();
}

void Robot::loadRobotModel(const std::string &description)
{
    robot_model_loader::RobotModelLoader::Options options(description);
    options.load_kinematics_solvers_ = false;
    options.urdf_string_ = urdf_;
    options.srdf_string_ = srdf_;

    loader_.reset(new robot_model_loader::RobotModelLoader(options));
    kinematics_.reset(new kinematics_plugin_loader::KinematicsPluginLoader(description));

    model_ = loader_->getModel();
}

bool Robot::loadKinematics(const std::string &group_name, bool load_subgroups)
{
    // Needs to be called first to read the groups defined in the SRDF from the ROS params.
    robot_model::SolverAllocatorFn allocator = kinematics_->getLoaderFunction(loader_->getSRDF());

    const auto &groups = kinematics_->getKnownGroups();
    if (groups.empty())
    {
        RBX_ERROR("No kinematics plugins defined. Fill and load kinematics.yaml!");
        return false;
    }

    if (!model_->hasJointModelGroup(group_name))
    {
        RBX_ERROR("No JMG defined for `%s`!", group_name);
        return false;
    }

    std::vector<std::string> load_names;

    // If requested, also attempt to load the kinematics solvers for subgroups.
    if (load_subgroups)
    {
        const auto &subgroups = model_->getJointModelGroup(group_name)->getSubgroupNames();
        load_names.insert(load_names.end(), subgroups.begin(), subgroups.end());
    }

    // Check if this group also has an associated kinematics solver to load.
    if (std::find(groups.begin(), groups.end(), group_name) != groups.end())
        load_names.emplace_back(group_name);

    auto timeout = kinematics_->getIKTimeout();

    for (const auto &name : load_names)
    {
        // Check if kinematics have already been loaded for this group.
        if (imap_.find(name) != imap_.end())
            continue;

        if (!model_->hasJointModelGroup(name) ||
            std::find(groups.begin(), groups.end(), name) == groups.end())
        {
            RBX_ERROR("No JMG or Kinematics defined for `%s`!", name);
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
                RBX_ERROR("Kinematics solver %s does not support joint group %s.  Error: %s",
                          typeid(*solver).name(), name, error_msg);
                return false;
            }
        }
        else
        {
            RBX_ERROR("Kinematics solver could not be instantiated for joint group `%s`.", name);
            return false;
        }

        RBX_INFO("Loaded Kinematics Solver for  `%s`", name);
        jmg->setDefaultIKTimeout(timeout[name]);
    }

    model_->setKinematicsAllocators(imap_);
    return true;
}

void Robot::setSRDFPostProcessAddPlanarJoint(const std::string &name)
{
    setSRDFPostProcessFunction([&, name](tinyxml2::XMLDocument &doc) -> bool {
        tinyxml2::XMLElement *virtual_joint = doc.NewElement("virtual_joint");
        virtual_joint->SetAttribute("name", name.c_str());
        virtual_joint->SetAttribute("type", "planar");
        virtual_joint->SetAttribute("parent_frame", "world");
        virtual_joint->SetAttribute("child_link", model_->getRootLink()->getName().c_str());

        doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

        return true;
    });
}

void Robot::setSRDFPostProcessAddFloatingJoint(const std::string &name)
{
    setSRDFPostProcessFunction([&, name](tinyxml2::XMLDocument &doc) -> bool {
        tinyxml2::XMLElement *virtual_joint = doc.NewElement("virtual_joint");
        virtual_joint->SetAttribute("name", name.c_str());
        virtual_joint->SetAttribute("type", "floating");
        virtual_joint->SetAttribute("parent_frame", "world");
        virtual_joint->SetAttribute("child_link", model_->getRootLink()->getName().c_str());

        doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

        return true;
    });
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

const std::string &Robot::getURDFString() const
{
    return urdf_;
}

srdf::ModelConstSharedPtr Robot::getSRDF() const
{
    return model_->getSRDF();
}

const std::string &Robot::getSRDFString() const
{
    return srdf_;
}

const robot_model::RobotStatePtr &Robot::getScratchStateConst() const
{
    return scratch_;
}

robot_model::RobotStatePtr &Robot::getScratchState()
{
    return scratch_;
}

robot_model::RobotStatePtr Robot::cloneScratchState() const
{
    auto state = allocState();
    *state = *scratch_;

    return state;
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

void Robot::setState(const sensor_msgs::JointState &state)
{
    scratch_->setVariableValues(state);
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

moveit_msgs::RobotState Robot::getStateMsg() const
{
    moveit_msgs::RobotState message;
    moveit::core::robotStateToRobotStateMsg(*scratch_, message);

    return message;
}

void Robot::setStateMsgGroupState(moveit_msgs::RobotState &state, const std::string &group,
                                  const std::vector<double> &positions) const
{
    robot_state::RobotState temp(getModelConst());
    moveit::core::robotStateMsgToRobotState(state, temp);

    auto *jmg = getModelConst()->getJointModelGroup(group);
    temp.setJointGroupPositions(jmg, positions);

    moveit::core::robotStateToRobotStateMsg(temp, state);
}

std::vector<std::string> Robot::getJointNames() const
{
    return scratch_->getVariableNames();
}

bool Robot::hasJoint(const std::string &joint) const
{
    const auto &joint_names = getJointNames();
    return (std::find(joint_names.begin(), joint_names.end(), joint) != joint_names.end());
}

//
// IKQuery
//

Robot::IKQuery::IKQuery(const std::string &group) : group(group)
{
}

Robot::IKQuery::IKQuery(const std::string &group, const std::string &tip,
                        const robot_state::RobotState &start, const Eigen::Vector3d &direction,
                        double distance)
  : IKQuery(group, tip, start, distance * direction.normalized())
{
}

Robot::IKQuery::IKQuery(const std::string &group, const std::string &tip,
                        const robot_state::RobotState &start, const Eigen::Vector3d &position_offset,
                        const Eigen::Quaterniond &rotation_offset)
  : IKQuery(group, tip, start, TF::createPoseQ(position_offset, rotation_offset))
{
}

Robot::IKQuery::IKQuery(const std::string &group, const std::string &tip,
                        const robot_state::RobotState &start, const RobotPose &offset)
  : IKQuery(group, start.getGlobalLinkTransform(tip) * offset)
{
}

Robot::IKQuery::IKQuery(const std::string &group,   //
                        const RobotPose &offset,    //
                        const ScenePtr &scene,      //
                        const std::string &object,  //
                        const Eigen::Vector3d &tolerances,
                        bool verbose)
  : group(group), scene(scene), verbose(verbose)
{
    const auto &pose = scene->getObjectGraspPose(object, offset);
    addRequest("",                                     //
               Geometry::makeBox(tolerances),          //
               TF::createPoseXYZ(pose.translation()),  //
               Eigen::Quaterniond(pose.rotation()));
}

Robot::IKQuery::IKQuery(const std::string &group, const RobotPose &pose, double radius,
                        const Eigen::Vector3d &tolerance)
  : IKQuery(group,                                //
            pose.translation(),                   //
            Eigen::Quaterniond{pose.rotation()},  //
            radius,                               //
            tolerance)
{
}

Robot::IKQuery::IKQuery(const std::string &group,                                                //
                        const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,  //
                        double radius, const Eigen::Vector3d &tolerance)
  : IKQuery(group,                         //
            Geometry::makeSphere(radius),  //
            TF::createPoseXYZ(position),   //
            orientation,                   //
            tolerance)
{
}

Robot::IKQuery::IKQuery(const std::string &group, const GeometryConstPtr &region, const RobotPose &pose,
                        const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerance,
                        const ScenePtr &scene, bool verbose)
  : group(group), scene(scene), verbose(verbose)
{
    addRequest("", region, pose, orientation, tolerance);
}

Robot::IKQuery::IKQuery(const std::string &group, const moveit_msgs::PositionConstraint &pc,
                        const moveit_msgs::OrientationConstraint &oc)
  : group(group)
{
    if (pc.link_name != oc.link_name)
        throw Exception(
            1, log::format("Link name mismatch in constraints, `%1%` != `%2%`", pc.link_name, oc.link_name));

    if (not TF::isVecZero(TF::vectorMsgToEigen(pc.target_point_offset)))
        throw Exception(1, "target_point_offset in position constraint not supported.");

    const auto &cr = pc.constraint_region;

    if (not cr.meshes.empty())
        throw Exception(1, "Cannot specify mesh regions!");

    if (cr.primitives.size() > 1)
        throw Exception(1, "Cannot specify more than one primitive!");

    const auto &region = Geometry::makeSolidPrimitive(cr.primitives[0]);
    const auto &pose = TF::poseMsgToEigen(cr.primitive_poses[0]);

    const auto &rotation = TF::quaternionMsgToEigen(oc.orientation);
    Eigen::Vector3d tolerance{oc.absolute_x_axis_tolerance,  //
                              oc.absolute_y_axis_tolerance,  //
                              oc.absolute_z_axis_tolerance};

    addRequest(pc.link_name, region, pose, rotation, tolerance);
}

Robot::IKQuery::IKQuery(const std::string &group, const RobotPoseVector &poses,
                        const std::vector<std::string> &input_tips, double radius,
                        const Eigen::Vector3d &tolerance, const ScenePtr &scene, bool verbose)
  : group(group), scene(scene), verbose(verbose)
{
    if (poses.size() != input_tips.size())
        throw Exception(1, "Invalid multi-target IK query. poses != tips.");

    for (std::size_t i = 0; i < poses.size(); ++i)
        addRequest(input_tips[i],                              //
                   Geometry::makeSphere(radius),               //
                   TF::createPoseXYZ(poses[i].translation()),  //
                   Eigen::Quaterniond{poses[i].rotation()},    //
                   tolerance);
}

Robot::IKQuery::IKQuery(const std::string &group, const std::vector<std::string> &input_tips,
                        const std::vector<GeometryConstPtr> &regions, const RobotPoseVector &poses,
                        const std::vector<Eigen::Quaterniond> &orientations,
                        const EigenSTL::vector_Vector3d &tolerances, const ScenePtr &scene, bool verbose)
  : group(group), scene(scene), verbose(verbose)
{
    if (poses.size() != input_tips.size()       //
        or poses.size() != regions.size()       //
        or poses.size() != orientations.size()  //
        or poses.size() != tolerances.size())
        throw Exception(1, "Invalid multi-target IK query. Vectors are of different length.");

    for (std::size_t i = 0; i < poses.size(); ++i)
        addRequest(input_tips[i], regions[i], poses[i], orientations[i], tolerances[i]);
}

void Robot::IKQuery::addRequest(const std::string &tip, const GeometryConstPtr &region, const RobotPose &pose,
                                const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerance)
{
    tips.emplace_back(tip);
    regions.emplace_back(region);
    region_poses.emplace_back(pose);
    orientations.emplace_back(orientation);
    tolerances.emplace_back(tolerance);
}

void Robot::IKQuery::setScene(const ScenePtr &scene_in, bool verbose_in)
{
    scene = scene_in;
    verbose = verbose_in;
}

void Robot::IKQuery::addMetric(const Metric &metric_function)
{
    metrics.emplace_back(metric_function);
}

void Robot::IKQuery::addDistanceMetric(double weight)
{
    addMetric([weight](const robot_state::RobotState &state, const SceneConstPtr &scene,
                       const kinematic_constraints::ConstraintEvaluationResult &result) {
        return weight * result.distance;
    });
}

void Robot::IKQuery::addCenteringMetric(double weight)
{
    addMetric([&, weight](const robot_state::RobotState &state, const SceneConstPtr &scene,
                          const kinematic_constraints::ConstraintEvaluationResult &result) {
        const auto &jmg = state.getJointModelGroup(group);
        const auto &min = state.getMinDistanceToPositionBounds(jmg);
        double extent = min.second->getMaximumExtent() / 2.;
        return weight * (extent - min.first) / extent;
    });
}

void Robot::IKQuery::addClearanceMetric(double weight)
{
    addMetric([&, weight](const robot_state::RobotState &state, const SceneConstPtr &scene,
                          const kinematic_constraints::ConstraintEvaluationResult &result) {
        if (scene)
        {
            double v = scene->distanceToCollision(state);
            return weight * v;
        }

        return 0.;
    });
}

bool Robot::IKQuery::sampleRegion(RobotPose &pose, std::size_t index) const
{
    const auto &point = regions[index]->sample();
    if (point.first)
    {
        pose = region_poses[index];
        pose.translate(point.second);
        pose.rotate(TF::sampleOrientation(orientations[index], tolerances[index]));
    }

    return point.first;
}

bool Robot::IKQuery::sampleRegions(RobotPoseVector &poses) const
{
    const std::size_t n = regions.size();
    poses.resize(n);

    bool sampled = true;
    for (std::size_t j = 0; j < n and sampled; ++j)
        sampled &= sampleRegion(poses[j], j);

    return sampled;
}

void Robot::IKQuery::getMessage(const std::string &base_frame, moveit_msgs::Constraints &msg) const
{
    const std::size_t n = regions.size();

    msg.name = "IKQuery";

    for (std::size_t i = 0; i < n; ++i)
    {
        auto pos = TF::getPositionConstraint(tips[i], base_frame, region_poses[i], regions[i]);
        auto orn = TF::getOrientationConstraint(tips[i], base_frame, orientations[i], tolerances[i]);
        pos.weight = 1. / n;
        orn.weight = 1. / n / 2.;

        msg.position_constraints.emplace_back(pos);
        msg.orientation_constraints.emplace_back(orn);
    }
}

kinematic_constraints::KinematicConstraintSetPtr Robot::IKQuery::getAsConstraints(const Robot &robot) const
{
    moveit_msgs::Constraints msg;
    const auto &root = robot.getModelConst()->getRootLink()->getName();
    getMessage(root, msg);

    auto constraints = std::make_shared<kinematic_constraints::KinematicConstraintSet>(robot.getModelConst());
    moveit::core::Transforms none(root);

    constraints->add(msg, none);

    return constraints;
}

double Robot::IKQuery::getMetricValue(const robot_state::RobotState &state,
                                      const kinematic_constraints::ConstraintEvaluationResult &result) const
{
    double v = 0.;
    for (const auto &metric : metrics)
        v += metric(state, scene, result);

    return v;
}

bool Robot::setFromIK(const IKQuery &query)
{
    return setFromIK(query, *scratch_);
}

bool Robot::setFromIK(const IKQuery &query, robot_state::RobotState &state) const
{
    // copy query for unconstness
    IKQuery query_copy(query);

    // If there are no tips in the query,
    // we use the tip frame from the original query group
    if (query_copy.tips[0].empty())
        query_copy.tips = getSolverTipFrames(query.group);

    const robot_model::JointModelGroup *jmg = model_->getJointModelGroup(query_copy.group);
    const auto &gsvcf = (query_copy.scene) ? query_copy.scene->getGSVCF(query_copy.verbose) :
                                             moveit::core::GroupStateValidityCallbackFn{};

    bool evaluate = not query_copy.metrics.empty() or query_copy.validate;
    kinematic_constraints::ConstraintEvaluationResult result;

    const auto &constraints = (evaluate) ? query_copy.getAsConstraints(*this) : nullptr;

    // Best state if evaluating metrics.
    auto best = (query_copy.metrics.empty()) ? nullptr : std::make_shared<robot_state::RobotState>(state);
    double best_value = constants::inf;

    bool success = false;
    RobotPoseVector targets;
    for (std::size_t i = 0; i < query_copy.attempts and not success; ++i)
    {
        // Sample new target poses from regions.
        query_copy.sampleRegions(targets);

#if ROBOWFLEX_AT_LEAST_MELODIC
        // Multi-tip IK: Will delegate automatically to RobotState::setFromIKSubgroups() if the kinematics
        // solver doesn't support multi-tip queries.
        success =
            state.setFromIK(jmg, targets, query_copy.tips, query_copy.timeout, gsvcf, query_copy.options);
#else
        // attempts was a prior field that was deprecated in melodic
        success =
            state.setFromIK(jmg, targets, query_copy.tips, 1, query_copy.timeout, gsvcf, query_copy.options);
#endif

        if (evaluate)
        {
            state.update();
            result = constraints->decide(state, query_copy.verbose);
        }

        // Externally validate result
        if (query_copy.validate)
        {
            if (query_copy.verbose)
                RBX_INFO("Constraint Distance: %1%", result.distance);

            bool no_collision =
                (query_copy.scene) ? not query_copy.scene->checkCollision(state).collision : true;

            success =             //
                no_collision and  //
                ((query_copy.valid_distance > 0.) ? result.distance <= query_copy.valid_distance :
                                                    result.satisfied);
        }

        // If success, evaluate state for metrics.
        if (success and not query_copy.metrics.empty())
        {
            double v = query_copy.getMetricValue(state, result);

            if (query_copy.verbose)
                RBX_INFO("State Metric Value: %1%", v);

            if (v < best_value)
            {
                if (query_copy.verbose)
                    RBX_INFO("State is current best!");

                best_value = v;
                *best = state;
            }

            success = false;
        }

        if (query_copy.random_restart and not success)
            state.setToRandomPositions(jmg);
    }

    // If evaluating metric, copy best state.
    if (std::isfinite(best_value))
    {
        state = *best;
        success = true;
    }

    state.update();
    return success;
}

bool Robot::validateIKQuery(const IKQuery &query, const robot_state::RobotState &state) const
{
    const auto &constraints = query.getAsConstraints(*this);
    const auto &result = constraints->decide(state, query.verbose);
    return (query.valid_distance > 0.) ? result.distance <= query.valid_distance : result.satisfied;
}

double Robot::distanceToIKQuery(const IKQuery &query, const robot_state::RobotState &state) const
{
    const auto &constraints = query.getAsConstraints(*this);
    const auto &result = constraints->decide(state, query.verbose);
    return result.distance;
}

const RobotPose &Robot::getLinkTF(const std::string &name) const
{
    return scratch_->getGlobalLinkTransform(name);
}

const RobotPose Robot::getRelativeLinkTF(const std::string &base, const std::string &target) const
{
    auto base_tf = scratch_->getGlobalLinkTransform(base);
    auto target_tf = scratch_->getGlobalLinkTransform(target);

    return base_tf.inverse() * target_tf;
}

bool Robot::toYAMLFile(const std::string &file) const
{
    moveit_msgs::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(*scratch_, msg);

    const auto &yaml = IO::toNode(msg);
    return IO::YAMLToFile(yaml, file);
}

robot_model::RobotStatePtr Robot::allocState() const
{
    // No make_shared() for indigo compatibility
    robot_state::RobotStatePtr state;
    state.reset(new robot_state::RobotState(getModelConst()));
    state->setToDefaultValues();

    return state;
}

std::vector<std::string> Robot::getSolverTipFrames(const std::string &group) const
{
    const auto &jmg = model_->getJointModelGroup(group);
    const auto &solver = jmg->getSolverInstance();
    if (solver)
        return solver->getTipFrames();

    return {};
}

std::string Robot::getSolverBaseFrame(const std::string &group) const
{
    const auto &jmg = model_->getJointModelGroup(group);
    const auto &solver = jmg->getSolverInstance();
    if (solver)
        return solver->getBaseFrame();

    return "";
}

namespace
{
    YAML::Node addLinkGeometry(const urdf::GeometrySharedPtr &geometry, bool resolve = true)
    {
        YAML::Node node;
        switch (geometry->type)
        {
            case urdf::Geometry::MESH:
            {
                const auto &mesh = static_cast<urdf::Mesh *>(geometry.get());
                node["type"] = "mesh";

                if (resolve)
                    node["resource"] = IO::resolvePath(mesh->filename);
                else
                    node["resource"] = mesh->filename;

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
        origin["orientation"] =
            std::vector<double>({pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w});
        node["origin"] = origin;
        ROBOWFLEX_YAML_FLOW(node["origin"]);
    }

    YAML::Node addLinkVisual(const urdf::VisualSharedPtr &visual, bool resolve = true)
    {
        YAML::Node node;
        if (visual)
        {
            if (visual->geometry)
            {
                const auto &geometry = visual->geometry;
                node = addLinkGeometry(geometry, resolve);

                if (visual->material)
                {
                    const auto &material = visual->material;
                    addLinkMaterial(node, material);
                }

                const auto &pose = visual->origin;

                Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
                Eigen::Quaterniond rotation(pose.rotation.w,  //
                                            pose.rotation.x, pose.rotation.y, pose.rotation.z);
                Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

                // TODO: Also check if rotation is not zero.
                if (position.norm() > 0 || rotation.angularDistance(identity) > 0)
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

    RobotPose urdfPoseToEigen(const urdf::Pose &pose)
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
                RobotPose tf =
                    state->getGlobalLinkTransform(link);  // * urdfPoseToEigen(urdf_link->visual->origin);
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

bool Robot::dumpToScene(const std::string &filename) const
{
    YAML::Node node;

    const auto &urdf = model_->getURDF();
    for (const auto &link_name : model_->getLinkModelNames())
    {
        const auto &urdf_link = urdf->getLink(link_name);
        if (urdf_link->visual)
        {
            const auto &link = model_->getLinkModel(link_name);

            std::vector<YAML::Node> visuals;
#if ROBOWFLEX_AT_LEAST_KINETIC
            for (const auto &element : urdf_link->visual_array)
                if (element)
                    visuals.emplace_back(addLinkVisual(element, false));
#else
            if (urdf_link->visual)
                visuals.push_back(addLinkVisual(urdf_link->visual, false));
#endif

            YAML::Node object;
            object["id"] = link->getName();

            YAML::Node meshes;
            YAML::Node mesh_poses;
            YAML::Node primitives;
            YAML::Node primitive_poses;

            RobotPose tf = scratch_->getGlobalLinkTransform(link);
            for (const auto &visual : visuals)
            {
                RobotPose pose = tf;
                if (IO::isNode(visual["origin"]))
                {
                    RobotPose offset = TF::poseMsgToEigen(IO::poseFromNode(visual["origin"]));
                    pose = pose * offset;
                }

                const auto &posey = IO::toNode(TF::poseEigenToMsg(pose));

                const auto &type = visual["type"].as<std::string>();
                if (type == "mesh")
                {
                    YAML::Node mesh;
                    mesh["resource"] = visual["resource"];
                    if (IO::isNode(visual["dimensions"]))
                        mesh["dimensions"] = visual["dimensions"];
                    else
                    {
                        mesh["dimensions"].push_back(1.);
                        mesh["dimensions"].push_back(1.);
                        mesh["dimensions"].push_back(1.);
                    }

                    ROBOWFLEX_YAML_FLOW(mesh["dimensions"]);

                    meshes.push_back(mesh);
                    mesh_poses.push_back(posey);
                }
                else
                {
                    YAML::Node primitive;
                    primitive["type"] = type;
                    primitive["dimensions"] = visual["dimensions"];
                    primitive_poses.push_back(posey);
                }
            }

            if (meshes.size())
            {
                object["meshes"] = meshes;
                object["mesh_poses"] = mesh_poses;
            }

            if (primitives.size())
            {
                object["primitives"] = primitives;
                object["primitive_poses"] = primitive_poses;
            }

            node["collision_objects"].push_back(object);
        }
    }

    YAML::Node scene;
    scene["world"] = node;

    return IO::YAMLToFile(scene, filename);
}

///
/// robowflex::ParamRobot
///

ParamRobot::ParamRobot(const std::string &name) : Robot(name)
{
    // Retrieve values from parameter server.
    handler_.getParam(ROBOT_DESCRIPTION, urdf_);
    handler_.getParam(ROBOT_DESCRIPTION + ROBOT_SEMANTIC, srdf_);

    initializeInternal(false);
}
