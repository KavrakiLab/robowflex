#include "robowflex.h"

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
    if (!handler_.hasParam(ROBOT_DESCRIPTION))
    {
        const std::string urdf_string = IO::loadXMLToString(urdf_file);
        if (urdf_string.empty())
        {
            ROS_ERROR("Failed to load URDF.");
            return false;
        }

        handler_.setParam(ROBOT_DESCRIPTION, urdf_string);
    }

    if (!handler_.hasParam(ROBOT_DESCRIPTION + ROBOT_SEMANTIC))
    {
        const std::string srdf_string = IO::loadXMLToString(srdf_file);
        if (srdf_string.empty())
        {
            ROS_ERROR("Failed to load SRDF.");
            return false;
        }

        handler_.setParam(ROBOT_DESCRIPTION + ROBOT_SEMANTIC, srdf_string);
    }

    if (!handler_.hasParam(ROBOT_DESCRIPTION + ROBOT_PLANNING))
    {
        auto &limits = IO::loadFileToYAML(limits_file);
        if (!limits.first)
        {
            ROS_ERROR("Failed to load joint limits.");
            return false;
        }

        handler_.loadYAMLtoROS(limits.second, ROBOT_DESCRIPTION + ROBOT_PLANNING);
    }

    if (!handler_.hasParam(ROBOT_DESCRIPTION + ROBOT_KINEMATICS))
    {
        auto &kinematics = IO::loadFileToYAML(kinematics_file);
        if (!kinematics.first)
        {
            ROS_ERROR("Failed to load kinematics.");
            return false;
        }

        handler_.loadYAMLtoROS(kinematics.second, ROBOT_DESCRIPTION + ROBOT_KINEMATICS);
    }

    return true;
}

void Robot::loadRobotModel()
{
    robot_model_loader::RobotModelLoader::Options options(handler_.getNamespace() + "/" + ROBOT_DESCRIPTION);
    options.load_kinematics_solvers_ = false;

    loader_ = robot_model_loader::RobotModelLoader(options);
    kinematics_.reset(new kinematics_plugin_loader::KinematicsPluginLoader(loader_.getRobotDescription()));

    model_ = std::move(loader_.getModel());
}

bool Robot::loadKinematics(const std::string &name)
{
    if (imap_.find(name) != imap_.end())
        return true;

    robot_model::SolverAllocatorFn allocator = kinematics_->getLoaderFunction(loader_.getSRDF());

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
