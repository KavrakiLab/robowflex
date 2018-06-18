#include <robowflex_library/robowflex.h>

using namespace robowflex;

const std::string Robot::ROBOT_DESCRIPTION = "robot_description";
const std::string Robot::ROBOT_SEMANTIC = "_semantic";
const std::string Robot::ROBOT_PLANNING = "_planning";
const std::string Robot::ROBOT_KINEMATICS = "_kinematics";

Robot::Robot(const std::string &name) : name_(name), handler_(name_)
{
}

bool Robot::initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                       const std::string &kinematics_file)
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
            ROS_ERROR("Kinematics solver %s does not support joint group %s.  Error: %s", typeid(*solver).name(),
                      name.c_str(), error_msg.c_str());
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

void Robot::setState(const std::vector<double> &positions)
{
    scratch_->setVariablePositions(positions);
}

void Robot::setFromIK(const std::string &group,                             //
                      const Geometry &region, const Eigen::Affine3d &pose,  //
                      const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances)
{
    Eigen::Affine3d sampled_pose = pose;

    sampled_pose.translate(region.sample());
    sampled_pose.rotate(TF::sampleOrientation(orientation, tolerances));

    geometry_msgs::Pose msg = TF::poseEigenToMsg(sampled_pose);

    robot_model::JointModelGroup *jmg = model_->getJointModelGroup(group);
    scratch_->setFromIK(jmg, msg);
}

const Eigen::Affine3d &Robot::getLinkTF(const std::string &name) const
{
    return scratch_->getGlobalLinkTransform(name);
}
