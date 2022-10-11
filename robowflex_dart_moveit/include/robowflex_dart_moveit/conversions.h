

/** \brief Convert a robowflex::Robot into a Dart robot.
 *  \param[in] robot Robot to convert.
 */
Robot(robowflex::RobotPtr robot);

Robot::Robot(const std::string &name, const ScenePtr &scene) : Structure(name, scene)
{
}

            /** \name State Operations
                \{ */

            /** \brief Set the current state of this robot from a MoveIt message.
             *  \param[in] msg Message to set state to.
             */
            void setStateFromMoveItMsg(const moveit_msgs::RobotState &msg);

            /** \brief Set a MoveIt message from the current state of this robot.
             *  \param[out] msg Message to set.
             */
            void setMoveItMsgFromState(moveit_msgs::RobotState &msg) const;

            /** \brief Set the current state of this robot from a MoveIt robot state.
             *  \param[in] state MoveIt state to set state to.
             */
            void setStateFromMoveItState(const robot_state::RobotState &state);

            /** \brief Set a MoveIt robot state from the current state of this robot.
             *  \param[out] state MoveIt state to set state to.
             */
            void setMoveItStateFromState(robot_state::RobotState &state) const;

            /** \brief Set the state using a MoveIt Joint Group.
             *  \param[in] jmg Name of joint group.
             *  \param[in] joints Value of Joint Group to set.
             */
            void setStateFromMoveItJMG(const std::string &jmg, const std::vector<double> &joints);

            /** \brief Set the state using a MoveIt Joint Group.
             *  \param[in] jmg Name of joint group.
             *  \param[in] vec Value of Joint Group to set.
             */
            void setStateFromMoveItJMG(const std::string &jmg, const Eigen::Ref<const Eigen::VectorXd> &vec);

            /** \brief Set a MoveIt Joint Group state from the current state of the robot.
             *  \param[in] jmg Name of joint group.
             *  \param[out] joints Joint values to set.
             */
            void setMoveItJMGFromState(const std::string &jmg, std::vector<double> &joints) const;

            /** \brief Set a MoveIt Joint Group state from the current state of the robot.
             *  \param[in] jmg Name of joint group.
             *  \param[out] vec Joint values to set.
             */
            void setMoveItJMGFromState(const std::string &jmg, Eigen::Ref<Eigen::VectorXd> vec) const;

            /** \} */
