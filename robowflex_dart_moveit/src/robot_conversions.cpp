/* Author: Zachary Kingston */

#include <moveit/robot_state/conversions.h>

#include <robowflex_util/filesystem.h>
#include <robowflex_util/math.h>

#include <robowflex_moveit/utility/conversions.h>

#include <robowflex_dart_moveit/robot_conversions.h>

using namespace robowflex::darts;

RobotPtr conversions::fromMoveItRobot(const robowflex::RobotConstPtr &robot)
{
    auto drobot = std::make_shared<darts::Robot>(robot->getName());

    std::ofstream urdf_file;
    std::string urdf_filename = robowflex::IO::createTempFile(urdf_file);

    std::ofstream srdf_file;
    std::string srdf_filename = robowflex::IO::createTempFile(srdf_file);

    urdf_file << robot->getURDFString();
    urdf_file.close();

    srdf_file << robot->getSRDFString();
    srdf_file.close();

    drobot->loadURDF(urdf_filename);
    drobot->loadSRDF(srdf_filename);

    robowflex::IO::deleteFile(urdf_filename);
    robowflex::IO::deleteFile(srdf_filename);

    return drobot;
}

void conversions::setStateFromMoveItMsg(darts::RobotPtr &robot, const moveit_msgs::RobotState &msg)
{
    for (std::size_t i = 0; i < msg.joint_state.name.size(); ++i)
        robot->setJoint(msg.joint_state.name[i], msg.joint_state.position[i]);

    auto &skeleton = robot->getSkeleton();
    for (std::size_t i = 0; i < msg.multi_dof_joint_state.joint_names.size(); ++i)
    {
        auto *joint = skeleton->getJoint(msg.multi_dof_joint_state.joint_names[i]);
        auto *j = static_cast<dart::dynamics::FreeJoint *>(joint);

        auto tfmsg = msg.multi_dof_joint_state.transforms[i];

        Eigen::Isometry3d tf;
        tf.translation() = TF::vectorMsgToEigen(tfmsg.translation);
        tf.linear() = TF::quaternionMsgToEigen(tfmsg.rotation).toRotationMatrix();

        j->setRelativeTransform(tf);
    }
}

void conversions::setMoveItMsgFromState(const darts::RobotConstPtr &robot, moveit_msgs::RobotState &msg)
{
    msg = moveit_msgs::RobotState();

    const auto &skeleton = robot->getSkeletonConst();
    for (std::size_t i = 0; i < skeleton->getNumJoints(); ++i)
    {
        auto *joint = skeleton->getJoint(i);

        // ignore fixed joints
        if (joint->getNumDofs() == 0)
            continue;

        const auto *j = dynamic_cast<dart::dynamics::FreeJoint *>(joint);
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

void conversions::setStateFromMoveItState(darts::RobotPtr &robot, const robot_state::RobotState &state)
{
    moveit_msgs::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(state, msg);
    setStateFromMoveItMsg(robot, msg);
}

void conversions::setMoveItStateFromState(const darts::RobotConstPtr &robot, robot_state::RobotState &state)
{
    moveit_msgs::RobotState msg;
    setMoveItMsgFromState(robot, msg);
    moveit::core::robotStateMsgToRobotState(msg, state);
}

void conversions::setStateFromMoveItJMG(darts::RobotPtr &robot, robot_state::RobotState &scratch,
                                        const std::string &jmg, const std::vector<double> &joints)
{
    setMoveItStateFromState(robot, scratch);      // copy current state
    scratch.setJointGroupPositions(jmg, joints);  // set only JMG state
    setStateFromMoveItState(robot, scratch);      // copy back
}

void conversions::setStateFromMoveItJMG(darts::RobotPtr &robot, robot_state::RobotState &scratch,
                                        const std::string &jmg, const Eigen::Ref<const Eigen::VectorXd> &vec)
{
    setMoveItStateFromState(robot, scratch);          // copy current state
    scratch.setJointGroupPositions(jmg, vec.data());  // set only JMG state
    setStateFromMoveItState(robot, scratch);          // copy back
}

void conversions::setMoveItJMGFromState(const darts::RobotConstPtr &robot, robot_state::RobotState &scratch,
                                        const std::string &jmg, std::vector<double> &joints)
{
    setMoveItStateFromState(robot, scratch);       // copy current state
    scratch.copyJointGroupPositions(jmg, joints);  // copy JMG state
}

void conversions::setMoveItJMGFromState(const darts::RobotConstPtr &robot, robot_state::RobotState &scratch,
                                        const std::string &jmg, Eigen::Ref<Eigen::VectorXd> vec)
{
    setMoveItStateFromState(robot, scratch);           // copy current state
    scratch.copyJointGroupPositions(jmg, vec.data());  // copy JMG state
}
