/* Author: Zachary Kingston */

#include <tinyxml2.h>

#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

#include <robowflex_library/robot.h>
#include <robowflex_library/io.h>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/acm.h>

using namespace robowflex::darts;

///
/// Robot
///

Robot::Robot(const std::string &name) : Structure(name)
{
}

Robot::Robot(robowflex::RobotPtr robot) : Structure(robot->getName())
{
    std::ofstream urdf_file;
    std::string urdf_filename = robowflex::IO::createTempFile(urdf_file);

    std::ofstream srdf_file;
    std::string srdf_filename = robowflex::IO::createTempFile(srdf_file);

    urdf_file << robot->getURDFString();
    urdf_file.close();

    srdf_file << robot->getSRDFString();
    srdf_file.close();

    loadURDF(urdf_filename);
    loadSRDF(srdf_filename);

    robowflex::IO::deleteFile(urdf_filename);
    robowflex::IO::deleteFile(srdf_filename);
}

RobotPtr Robot::clone(const std::string &newName) const
{
    auto robot = std::make_shared<Robot>(newName);
    robot->setSkeleton(skeleton_->cloneSkeleton());

    for (const auto &pair : acm_->getDisabledPairsConst())
        robot->getACM()->disableCollision(pair.first, pair.second);

    robot->getGroups() = groups_;

    return robot;
}

bool Robot::loadURDF(const std::string &urdf)
{
    return IO::loadURDF(*this, urdf);
}

bool Robot::addNameToGroup(const std::string &group, const std::string &name)
{
    if (not isGroup(group))
    {
        // SE3EZ_DEBUG("Adding joint %1% to group %2%.", name, group);
        groups_.emplace(group, std::vector<std::string>{name});
        return true;
    }

    auto &names = getGroupJointNames(group);

    for (const auto &item : names)
        if (item == name)
            return false;

    // SE3EZ_DEBUG("Adding joint %1% to group %2%.", name, group);
    names.emplace_back(name);
    return true;
}

bool Robot::addJointToGroup(const std::string &group, const std::string &joint_name)
{
    const auto &joint = skeleton_->getJoint(joint_name);
    if (not joint)
    {
        // SE3EZ_ERROR("Joint %1% not in skeleton.", joint_name);
        return false;
    }

    if (joint->getNumDofs() > 0)
        addNameToGroup(group, joint_name);

    return true;
}

bool Robot::addLinkToGroup(const std::string &group, const std::string &link_name)
{
    const auto &node = skeleton_->getBodyNode(link_name);
    if (not node)
    {
        // SE3EZ_ERROR("Link %1% not in skeleton.", link_name);
        return false;
    }

    const auto &joint = node->getParentJoint();
    if (not joint)
    {
        // SE3EZ_ERROR("Link %1% has no parent joint", link_name);
        return false;
    }

    // Only include mobile joints
    if (joint->getNumDofs() > 0)
        addNameToGroup(group, joint->getName());

    return true;
}

bool Robot::addChainToGroup(const std::string &group, const std::string &tip, const std::string &base)
{
    auto *node = skeleton_->getBodyNode(tip);
    if (not node)
    {
        // SE3EZ_ERROR("Tip link %1% not in skeleton.", tip);
        return false;
    }

    std::vector<std::string> names;
    while (node and node->getName() != base)
    {
        const auto &joint = node->getParentJoint();
        if (not joint)
        {
            // SE3EZ_ERROR("Link %1% has no parent joint", node->getName());
            return false;
        }

        if (joint->getNumDofs() > 0)
            names.emplace_back(joint->getName());

        node = joint->getParentBodyNode();
    }

    if (not node)
    {
        // SE3EZ_ERROR("Base link %1% not parent of tip link %2%", base, tip);
        return false;
    }

    for (const auto &name : names)
        addNameToGroup(group, name);

    return true;
}

bool Robot::addGroupToGroup(const std::string &group, const std::string &other)
{
    if (not isGroup(other))
    {
        // SE3EZ_ERROR("Group %1% does not exist", other);
        return false;
    }

    for (const auto &name : getGroupJointNamesConst(other))
        addNameToGroup(group, name);

    return true;
}

bool Robot::loadSRDF(const std::string &srdf)
{
    const auto &file = IO::getPackageFile(srdf);
    if (file.empty())
    {
        // SE3EZ_ERROR("File %1% cannot be found!", srdf);
        return false;
    }

    tinyxml2::XMLDocument doc;
    doc.LoadFile(file.c_str());

    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (not root)
    {
        // SE3EZ_ERROR("No child element in SRDF");
        return false;
    }

    // Parse groups
    tinyxml2::XMLElement *group = root->FirstChildElement("group");
    while (group != nullptr)
    {
        const auto &group_name = group->Attribute("name");

        tinyxml2::XMLElement *joints = group->FirstChildElement("joint");
        while (joints != nullptr)
        {
            const auto &joint_name = joints->Attribute("name");
            addJointToGroup(group_name, joint_name);

            joints = joints->NextSiblingElement("joint");
        }

        tinyxml2::XMLElement *links = group->FirstChildElement("link");
        while (links != nullptr)
        {
            const auto &link_name = links->Attribute("name");
            addLinkToGroup(group_name, link_name);

            links = links->NextSiblingElement("link");
        }

        tinyxml2::XMLElement *chains = group->FirstChildElement("chain");
        while (chains != nullptr)
        {
            const auto &base_link = chains->Attribute("base_link");
            const auto &tip_link = chains->Attribute("tip_link");
            addChainToGroup(group_name, tip_link, base_link);

            chains = chains->NextSiblingElement("chain");
        }

        tinyxml2::XMLElement *groups = group->FirstChildElement("group");
        while (groups != nullptr)
        {
            const auto &other_name = groups->Attribute("name");
            addGroupToGroup(group_name, other_name);

            groups = groups->NextSiblingElement("group");
        }

        processGroup(group_name);

        group = group->NextSiblingElement("group");
    }

    // Parse allowed collisions
    tinyxml2::XMLElement *dc = root->FirstChildElement("disable_collisions");
    while (dc != nullptr)
    {
        const auto &l1 = dc->Attribute("link1");
        const auto &l2 = dc->Attribute("link2");

        acm_->disableCollision(l1, l2);

        dc = dc->NextSiblingElement("disable_collisions");
    }

    return true;
}

bool Robot::isGroup(const std::string &name) const
{
    return groups_.find(name) != groups_.end();
}

std::map<std::string, std::vector<std::string>> &Robot::getGroups()
{
    return groups_;
}

const std::map<std::string, std::vector<std::string>> &Robot::getGroupsConst()
{
    return groups_;
}

std::vector<std::string> &Robot::getGroupJointNames(const std::string &name)
{
    return groups_.find(name)->second;
}

const std::vector<std::string> &Robot::getGroupJointNamesConst(const std::string &name) const
{
    return groups_.find(name)->second;
}

std::vector<dart::dynamics::Joint *> Robot::getGroupJoints(const std::string &name) const
{
    std::vector<dart::dynamics::Joint *> joints;
    for (const auto &dof : groups_.find(name)->second)
        joints.emplace_back(skeleton_->getJoint(dof));

    return joints;
}

const std::vector<std::size_t> &Robot::getGroupIndices(const std::string &name) const
{
    return group_indices_.find(name)->second;
}

void Robot::setDof(unsigned int index, double value)
{
    skeleton_->getDof(index)->setPosition(value);
}

std::size_t Robot::getNumDofsGroup(const std::string &name) const
{
    return getGroupIndices(name).size();
}

void Robot::getGroupState(const std::string &name, Eigen::Ref<Eigen::VectorXd> q) const
{
    q = skeleton_->getPositions(getGroupIndices(name));
}

void Robot::setGroupState(const std::string &name, const Eigen::Ref<const Eigen::VectorXd> &q)
{
    skeleton_->setPositions(getGroupIndices(name), q);
}

void Robot::processGroup(const std::string &name)
{
    std::vector<std::size_t> indices;

    auto joints = getGroupJoints(name);
    for (const auto &joint : joints)
    {
        for (std::size_t i = 0; i < joint->getNumDofs(); ++i)
            indices.emplace_back(joint->getDof(i)->getIndexInSkeleton());
    }

    group_indices_.emplace(name, indices);
}

RobotPtr robowflex::darts::loadMoveItRobot(const std::string &name, const std::string &urdf,
                                           const std::string &srdf)
{
    auto robot = std::make_shared<Robot>(name);
    robot->loadURDF(urdf);
    robot->loadSRDF(srdf);

    return robot;
}
