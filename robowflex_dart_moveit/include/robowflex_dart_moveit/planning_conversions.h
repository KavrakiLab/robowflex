
            /** \name MoveIt Planning Messages
                \{ */

            /** \brief Get workspace bounds from a planning request.
             *  \param[in] msg Planning request message.
             */
            void getWorkspaceBoundsFromMessage(const moveit_msgs::MotionPlanRequest &msg);

            /** \brief Set group for planning from message.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Planning request message.
             */
            void getGroupFromMessage(const std::string &robot_name,
                                     const moveit_msgs::MotionPlanRequest &msg);

            /** \brief Get the start state from the request message.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Planning request message.
             */
            void getStartFromMessage(const std::string &robot_name,
                                     const moveit_msgs::MotionPlanRequest &msg);

            /** \brief Get the set of path constraints from the request message.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Planning request message.
             */
            void getPathConstraintsFromMessage(const std::string &robot_name,
                                               const moveit_msgs::MotionPlanRequest &msg);

            /** \brief Get the goal constraints from the planning request message.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Planning request message.
             */
            ompl::base::GoalPtr getGoalFromMessage(const std::string &robot_name,
                                                   const moveit_msgs::MotionPlanRequest &msg);

            /** \brief Get a TSR from an position constraint.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Position constraint.
             */
            TSRPtr fromPositionConstraint(const std::string &robot_name,
                                          const moveit_msgs::PositionConstraint &msg) const;

            /** \brief Get a TSR from an orientation constraint.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Orientation constraint.
             */
            TSRPtr fromOrientationConstraint(const std::string &robot_name,
                                             const moveit_msgs::OrientationConstraint &msg) const;

            /** \brief Get a joint region goal from a message.
             *  \param[in] msgs Joint constraint messages to create region goal.
             *  \return The joint region goal.
             */
            JointRegionGoalPtr
            fromJointConstraints(const std::vector<moveit_msgs::JointConstraint> &msgs) const;

            /** \brief Use all of the planning request message to setup motion planning.
             *  \param[in] robot_name Robot name to use message on.
             *  \param[in] msg Planning request message.
             */
            ompl::base::GoalPtr fromMessage(const std::string &robot_name,
                                            const moveit_msgs::MotionPlanRequest &msg);

            /** \} */
