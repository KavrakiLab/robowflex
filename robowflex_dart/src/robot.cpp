/* Author: Zachary Kingston */

#include <tinyxml2.h>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>

#include <robowflex_library/io.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/tf.h>

#include <robowflex_dart/acm.h>
#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>

using namespace robowflex::darts;

///
/// Robot
///

Robot::Robot(const std::string &name) : Structure(name)
{
}

Robot::Robot(robowflex::RobotPtr robot) : Structure(robot->getName()), robot_(robot)
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

Robot::Robot(const std::string &name, const ScenePtr &scene) : Structure(name, scene)
{
}

RobotPtr Robot::cloneRobot(const std::string &newName) const
{
    auto robot = std::make_shared<Robot>(newName);
    robot->setSkeleton(skeleton_->cloneSkeleton(newName));

    for (const auto &pair : acm_->getDisabledPairsConst())
        robot->getACM()->disableCollision(pair.first, pair.second);

    robot->setGroups(groups_);
    robot->setNamedGroupStates(group_states_);

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
        RBX_DEBUG("Adding joint %1% to group %2%.", name, group);
        groups_.emplace(group, std::vector<std::string>{name});
        return true;
    }

    auto &names = getGroupJointNames(group);

    for (const auto &item : names)
        if (item == name)
            return false;

    RBX_DEBUG("Adding joint %1% to group %2%.", name, group);
    names.emplace_back(name);
    return true;
}

bool Robot::addJointToGroup(const std::string &group, const std::string &joint_name)
{
    const auto &joint = skeleton_->getJoint(joint_name);
    if (not joint)
    {
        RBX_ERROR("Joint %1% not in skeleton.", joint_name);
        return false;
    }

    if (joint->getNumDofs() > 0)
        addNameToGroup(group, joint_name);

    processGroup(group);
    return true;
}

bool Robot::addLinkToGroup(const std::string &group, const std::string &link_name)
{
    const auto &node = skeleton_->getBodyNode(link_name);
    if (not node)
    {
        RBX_ERROR("Link %1% not in skeleton.", link_name);
        return false;
    }

    const auto &joint = node->getParentJoint();
    if (not joint)
    {
        RBX_ERROR("Link %1% has no parent joint", link_name);
        return false;
    }

    if (joint->getName() == "rootJoint")
        return false;

    // Only include mobile joints
    if (joint->getNumDofs() > 0)
        addNameToGroup(group, joint->getName());

    processGroup(group);
    return true;
}

bool Robot::addChainToGroup(const std::string &group, const std::string &tip, const std::string &base)
{
    auto *node = skeleton_->getBodyNode(tip);
    if (not node)
    {
        RBX_ERROR("Tip link %1% not in skeleton.", tip);
        return false;
    }

    std::vector<std::string> names;
    while (node and node->getName() != base)
    {
        const auto &joint = node->getParentJoint();
        if (not joint)
        {
            RBX_ERROR("Link %1% has no parent joint", node->getName());
            return false;
        }

        if (joint->getNumDofs() > 0)
            names.emplace_back(joint->getName());

        node = joint->getParentBodyNode();
    }

    if (not node)
    {
        RBX_ERROR("Base link %1% not parent of tip link %2%", base, tip);
        return false;
    }

    for (const auto &name : names)
        addNameToGroup(group, name);

    processGroup(group);
    return true;
}

bool Robot::addGroupToGroup(const std::string &group, const std::string &other)
{
    if (not isGroup(other))
    {
        RBX_ERROR("Group %1% does not exist", other);
        return false;
    }

    for (const auto &name : getGroupJointNamesConst(other))
        addNameToGroup(group, name);

    processGroup(group);
    return true;
}

bool Robot::loadSRDF(const std::string &srdf)
{
    const auto &file = IO::getPackageFile(srdf);
    if (file.empty())
    {
        RBX_ERROR("File %1% cannot be found!", srdf);
        return false;
    }

    tinyxml2::XMLDocument doc;
    doc.LoadFile(file.c_str());

    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (not root)
    {
        RBX_ERROR("No child element in SRDF");
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
                RBX_ERROR("Couldn't find %s for virtual joint!", parent);
        }

        auto *cnode = skeleton_->getBodyNode(child);
        if (cnode)
        {
            if (type == "floating")
            {
                dart::dynamics::FreeJoint::Properties joint;
                joint.mName = name;

                cnode->moveTo<dart::dynamics::FreeJoint>(pnode, joint);
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

            auto values = robowflex::IO::tokenize<double>(value, " ");
            auto joint = getGroupJoint(group, jname);
            if (joint.second != nullptr)
            {
                for (std::size_t i = 0; i < values.size(); ++i)
                    state[joint.first + i] = values[i];
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

void Robot::setGroups(const GroupsMap &newGroups)
{
    groups_ = newGroups;
    for (const auto &pair : groups_)
        processGroup(pair.first);
}

const Robot::GroupsMap &Robot::getGroups() const
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

void Robot::setNamedGroupStates(const NamedStatesMap &states)
{
    group_states_ = states;
}

const Robot::NamedStatesMap &Robot::getNamedGroupStates() const
{
    return group_states_;
}

std::vector<std::string> Robot::getNamedGroupStates(const std::string & /*group*/) const
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

        return false;
    }

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

void Robot::setStateFromMoveItMsg(const moveit_msgs::RobotState &msg)
{
    for (std::size_t i = 0; i < msg.joint_state.name.size(); ++i)
        setJoint(msg.joint_state.name[i], msg.joint_state.position[i]);

    for (std::size_t i = 0; i < msg.multi_dof_joint_state.joint_names.size(); ++i)
    {
        auto *joint = skeleton_->getJoint(msg.multi_dof_joint_state.joint_names[i]);
        auto *j = static_cast<dart::dynamics::FreeJoint *>(joint);

        auto tfmsg = msg.multi_dof_joint_state.transforms[i];

        Eigen::Isometry3d tf;
        tf.translation() = TF::vectorMsgToEigen(tfmsg.translation);
        tf.linear() = TF::quaternionMsgToEigen(tfmsg.rotation).toRotationMatrix();

        j->setRelativeTransform(tf);
    }
}

void Robot::setMoveItMsgFromState(moveit_msgs::RobotState &msg) const
{
    msg = moveit_msgs::RobotState();

    for (std::size_t i = 0; i < skeleton_->getNumJoints(); ++i)
    {
        auto *joint = skeleton_->getJoint(i);

        // ignore fixed joints
        if (joint->getNumDofs() == 0)
            continue;

        auto *j = dynamic_cast<dart::dynamics::FreeJoint *>(joint);
        if (j)
        {
            msg.multi_dof_joint_state.joint_names.push_back(joint->getName());

            auto tf = joint->getRelativeTransform();
            geometry_msgs::Transform tfmsg;
            tfmsg.translation = TF::vectorEigenToMsg(tf.translation());
            tfmsg.rotation = TF::quaternionEigenToMsg(TF::getPoseRotation(tf));

            msg.multi_dof_joint_state.transforms.push_back(tfmsg);
        }
        else
        {
            msg.joint_state.name.push_back(joint->getName());
            msg.joint_state.position.push_back(joint->getPositions()[0]);
        }
    }
}

void Robot::setStateFromMoveItState(const robot_state::RobotState &state)
{
    moveit_msgs::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(state, msg);
    setStateFromMoveItMsg(msg);
}

void Robot::setMoveItStateFromState(robot_state::RobotState &state) const
{
    moveit_msgs::RobotState msg;
    setMoveItMsgFromState(msg);
    moveit::core::robotStateMsgToRobotState(msg, state);
}

void Robot::setStateFromMoveItJMG(const std::string &jmg, const std::vector<double> &joints)
{
    if (not robot_)
        return;

    setMoveItStateFromState(*robot_->getScratchState());             // copy current state
    robot_->getScratchState()->setJointGroupPositions(jmg, joints);  // set only JMG state
    setStateFromMoveItState(*robot_->getScratchState());             // copy back
}

void Robot::setStateFromMoveItJMG(const std::string &jmg, const Eigen::Ref<const Eigen::VectorXd> &vec)
{
    if (not robot_)
        return;

    setMoveItStateFromState(*robot_->getScratchState());                 // copy current state
    robot_->getScratchState()->setJointGroupPositions(jmg, vec.data());  // set only JMG state
    setStateFromMoveItState(*robot_->getScratchState());                 // copy back
}

void Robot::setMoveItJMGFromState(const std::string &jmg, std::vector<double> &joints) const
{
    if (not robot_)
        return;

    setMoveItStateFromState(*robot_->getScratchState());              // copy current state
    robot_->getScratchState()->copyJointGroupPositions(jmg, joints);  // copy JMG state
}

void Robot::setMoveItJMGFromState(const std::string &jmg, Eigen::Ref<Eigen::VectorXd> vec) const
{
    if (not robot_)
        return;

    setMoveItStateFromState(*robot_->getScratchState());                  // copy current state
    robot_->getScratchState()->copyJointGroupPositions(jmg, vec.data());  // copy JMG state
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

    auto it = group_indices_.find(group);
    if (it == group_indices_.end())
        group_indices_.emplace(group, indices);
    else
        it->second = indices;
}

RobotPtr robowflex::darts::loadMoveItRobot(const std::string &name, const std::string &urdf,
                                           const std::string &srdf)
{
    auto robot = std::make_shared<Robot>(name);
    robot->loadURDF(urdf);
    robot->loadSRDF(srdf);

    return robot;
}
