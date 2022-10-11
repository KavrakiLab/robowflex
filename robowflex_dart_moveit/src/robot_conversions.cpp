Robot::Robot(const std::string &name, const ScenePtr &scene) : Structure(name, scene)
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
