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

RobotPtr Robot::cloneRobot(const std::string &newName) const
{
    auto robot = std::make_shared<Robot>(newName);
    robot->setSkeleton(skeleton_->cloneSkeleton());

    for (const auto &pair : acm_->getDisabledPairsConst())
        robot->getACM()->disableCollision(pair.first, pair.second);

    robot->getGroups() = groups_;
    for (const auto &group : groups_)
        robot->processGroup(group.first);

    robot->getGroupStates() = group_states_;

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

    if (joint->getName() == "rootJoint")
        return false;

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

    // parse virtual joints
    tinyxml2::XMLElement *vj = root->FirstChildElement("virtual_joint");
    while (vj != nullptr)
    {
        std::string name = vj->Attribute("name");
        std::string type = vj->Attribute("type");
        std::string parent = vj->Attribute("parent_frame");
        std::string child = vj->Attribute("child_link");

        dart::dynamics::BodyNode *pnode = nullptr;
        if (parent != "world")
        {
            pnode = skeleton_->getBodyNode(parent);
            if (not pnode)
                std::cerr << "Couldn't find " << parent << " for virtual joint!" << std::endl;
        }

        auto cnode = skeleton_->getBodyNode(child);
        if (cnode)
        {
            if (type == "floating")
            {
                dart::dynamics::FreeJoint::Properties joint;
                joint.mName = name;

                cnode->moveTo<dart::dynamics::FreeJoint>(pnode, joint);
                // std::cout << cnode->getName() << " " << cnode->getParentJoint()->getName() << std::endl;
            }
        }

        vj = vj->NextSiblingElement("virtual_joint");
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

    // Parse named group states
    tinyxml2::XMLElement *gs = root->FirstChildElement("group_state");
    while (gs != nullptr)
    {
        const auto &group = gs->Attribute("group");
        const auto &name = gs->Attribute("name");

        Eigen::VectorXd state = Eigen::VectorXd::Zero(getNumDofsGroup(group));

        tinyxml2::XMLElement *js = gs->FirstChildElement("joint");
        while (js != nullptr)
        {
            const auto &value = js->Attribute("value");
            const auto &jname = js->Attribute("name");

            auto values = robowflex::IO::tokenize(value, " ");
            auto joint = getGroupJoint(group, jname);
            if (joint.second != nullptr)
            {
                for (std::size_t i = 0; i < values.size(); ++i)
                    state[joint.first + i] = std::stod(values[i]);
            }

            js = js->NextSiblingElement("joint");
        }

        setNamedGroupState(group, name, state);
        gs = gs->NextSiblingElement("group_state");
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

std::vector<std::string> &Robot::getGroupJointNames(const std::string &group)
{
    return groups_.find(group)->second;
}

const std::vector<std::string> &Robot::getGroupJointNamesConst(const std::string &group) const
{
    return groups_.find(group)->second;
}

std::vector<dart::dynamics::Joint *> Robot::getGroupJoints(const std::string &group) const
{
    std::vector<dart::dynamics::Joint *> joints;
    for (const auto &dof : groups_.find(group)->second)
        joints.emplace_back(skeleton_->getJoint(dof));

    return joints;
}

std::pair<std::size_t, dart::dynamics::Joint *> Robot::getGroupJoint(const std::string &group,
                                                                     const std::string &joint) const
{
    std::size_t i = 0;
    for (const auto &dof : getGroupJoints(group))
    {
        if (dof->getName() == joint)
            return {i, dof};

        i += dof->getNumDofs();
    }

    return {0, nullptr};
}

const std::vector<std::size_t> &Robot::getGroupIndices(const std::string &group) const
{
    return group_indices_.find(group)->second;
}

void Robot::setDof(unsigned int index, double value)
{
    skeleton_->getDof(index)->setPosition(value);
}

std::size_t Robot::getNumDofsGroup(const std::string &group) const
{
    return getGroupIndices(group).size();
}

void Robot::getGroupState(const std::string &group, Eigen::Ref<Eigen::VectorXd> q) const
{
    q = skeleton_->getPositions(getGroupIndices(group));
}

void Robot::setGroupState(const std::string &group, const Eigen::Ref<const Eigen::VectorXd> &q)
{
    skeleton_->setPositions(getGroupIndices(group), q);
}

std::map<std::string, std::map<std::string, Eigen::VectorXd>> &Robot::getGroupStates()
{
    return group_states_;
}

std::vector<std::string> Robot::getNamedGroupStates(const std::string &group) const
{
    std::vector<std::string> names;
    for (const auto &group : group_states_)
    {
        for (const auto &state : group.second)
            names.emplace_back(state.first);
    }

    return names;
}

bool Robot::getNamedGroupState(const std::string &group, const std::string &name,
                               Eigen::Ref<Eigen::VectorXd> q) const
{
    auto gsit = group_states_.find(group);
    if (gsit != group_states_.end())
    {
        auto it = gsit->second.find(name);
        if (it != gsit->second.end())
        {
            q = it->second;
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

void Robot::setNamedGroupState(const std::string &group, const std::string &name,
                               const Eigen::Ref<const Eigen::VectorXd> &q)
{
    auto gsit = group_states_.find(group);
    if (gsit == group_states_.end())
    {
        bool r;
        std::tie(gsit, r) = group_states_.emplace(group, std::map<std::string, Eigen::VectorXd>{});
    }

    auto &names = gsit->second;

    auto it = names.find(name);
    if (it == names.end())
    {
        bool r;
        std::tie(it, r) = names.emplace(name, q);
    }
    else
        it->second = q;
}

void Robot::processGroup(const std::string &group)
{
    std::vector<std::size_t> indices;

    auto joints = getGroupJoints(group);
    for (const auto &joint : joints)
    {
        for (std::size_t i = 0; i < joint->getNumDofs(); ++i)
            indices.emplace_back(joint->getDof(i)->getIndexInSkeleton());
    }

    group_indices_.emplace(group, indices);
}

RobotPtr robowflex::darts::loadMoveItRobot(const std::string &name, const std::string &urdf,
                                           const std::string &srdf)
{
    auto robot = std::make_shared<Robot>(name);
    robot->loadURDF(urdf);
    robot->loadSRDF(srdf);

    return robot;
}
