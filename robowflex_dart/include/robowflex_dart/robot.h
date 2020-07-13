/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_ROBOT_
#define ROBOWFLEX_DART_ROBOT_

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/simulation/World.hpp>

#include <robowflex_library/class_forward.h>

#include <robowflex_dart/io.h>
#include <robowflex_dart/structure.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot)
    ROBOWFLEX_CLASS_FORWARD(Scene)
    /** \endcond */

    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Robot)
        /** \endcond */

        /** \class robowflex::darts::TSRGoalPtr
            \brief A shared pointer wrapper for robowflex::darts::TSRGoal. */

        /** \class robowflex::darts::TSRGoalConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::TSRGoal. */

        /** \brief A sampleable goal region for OMPL for a set of TSRs.
         *  Samples goals in a separate thread using a clone of the world.
         */
        class Robot : public Structure
        {
        public:
            /** \brief A map of group name to member joints.
             */
            using GroupsMap = std::map<std::string, std::vector<std::string>>;

            /** \brief A map of group name to a map of state name to configurations.
             */
            using NamedStatesMap = std::map<std::string, std::map<std::string, Eigen::VectorXd>>;

            /** \name Constructors
                \{ */

            /** \brief Construct an empty robot.
             *  \param[in] name Name of the robot.
             */
            Robot(const std::string &name);

            /** \brief Convert a robowflex::Robot into a Dart robot.
             *  \param[in] robot Robot to convert.
             */
            Robot(robowflex::RobotPtr robot);

            /** \brief Convert a robowflex::Scene into a Dart robot.
             *  \param[in] name Name of the robot.
             *  \param[in] scene Scene to convert.
             */
            Robot(const std::string &name, const ScenePtr &scene);

            /** \brief Clone this robot with a new name.
             *  \param[in] newName Name for cloned robot.
             *  \return Cloned robot.
             */
            RobotPtr cloneRobot(const std::string &newName) const;

            /** \} */

            /** \name File Loading
                \{ */

            /** \brief Load a URDF into this robot.
             *  \param[in] urdf URDF filename to load. Can be package URI.
             *  \return True on success, false on failure.
             */
            bool loadURDF(const std::string &urdf);

            /** \brief Load a SRDF into this robot.
             *  \param[in] srdf SRDF filename to load. Can be package URI.
             *  \return True on success, false on failure.
             */
            bool loadSRDF(const std::string &srdf);

            /** \} */

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

            /** \name Group Operations
                \{ */

            /** \brief Get the groups currently in the robot. A map of group name to a list of all joints in
             * the group. \return The groups in the robot.
             */
            void setGroups(const GroupsMap &newGroups);

            /** \brief Get the groups currently in the robot. A map of group name to a list of all joints in
             * the group. \return The groups in the robot.
             */
            const GroupsMap &getGroups() const;

            /** \brief Get the joint names in a group.
             *  \param[in] group Group to get joint names for.
             *  \return The list of joint names.
             */
            std::vector<std::string> &getGroupJointNames(const std::string &group);

            /** \brief Get the joint names in a group.
             *  \param[in] group Group to get joint names for.
             *  \return The list of joint names.
             */
            const std::vector<std::string> &getGroupJointNamesConst(const std::string &group) const;

            /** \brief Get the joints for a group.
             *  \param[in] group Group to get joint names for.
             *  \return The list of joints.
             */
            std::vector<dart::dynamics::Joint *> getGroupJoints(const std::string &group) const;

            /** \brief Get a joint in a group along with its index in the group.
             *  \param[in] group Group to search.
             *  \param[in] joint Joint to find in group.
             *  \return The index of the joint and the joint in the group. {0, nullptr} if it does not exist.
             */
            std::pair<std::size_t, dart::dynamics::Joint *> getGroupJoint(const std::string &group,
                                                                          const std::string &joint) const;

            /** \brief Get the indices of joints corresponding to a group.
             *  \param[in] group Group to get indices for.
             *  \return The indices of the joints in a group.
             */
            const std::vector<std::size_t> &getGroupIndices(const std::string &group) const;

            /** \brief Checks if a group exists.
             *  \param[in] name Name of group to check.
             *  \return True if the group exists, false otherwise.
             */
            bool isGroup(const std::string &name) const;

            /** \brief Gets the number of DoFs controlled by a group.
             *  \param[in] group Group to get DoFs for.
             *  \return The number of DoFs in the group.
             */
            std::size_t getNumDofsGroup(const std::string &group) const;

            /** \brief Get the current state of the joints of a group.
             *  \param[in] group Group to get state for.
             *  \param[out] q Configuration of the group.
             */
            void getGroupState(const std::string &group, Eigen::Ref<Eigen::VectorXd> q) const;

            /** \brief Set the current state of the joints of a group.
             *  \param[in] group Group to set state for.
             *  \param[out] q Configuration to set the group.
             */
            void setGroupState(const std::string &group, const Eigen::Ref<const Eigen::VectorXd> &q);

            /** \} */

            /** \name Constructing Groups
                \{ */

            /** \brief Add a joint to a group.
             *  \param[in] group Group to add joint to.
             *  \param[in] joint_name Joint name to add.
             *  \return True on success, false on failure.
             */
            bool addJointToGroup(const std::string &group, const std::string &joint_name);

            /** \brief Add a the parent joint of a link to a group.
             *  \param[in] group Group to add joint to.
             *  \param[in] link_name Link to add.
             *  \return True on success, false on failure.
             */
            bool addLinkToGroup(const std::string &group, const std::string &link_name);

            /** \brief Add all the joints between two frames in a chain to a group.
             *  \param[in] group Group to add joint to.
             *  \param[in] tip Tip of the chain.
             *  \param[in] base Base of the chain.
             *  \return True on success, false on failure.
             */
            bool addChainToGroup(const std::string &group, const std::string &tip, const std::string &base);

            /** \brief Add all the joints in another group to this group.
             *  \param[in] group Group to add \a other to.
             *  \param[in] other Other group to add.
             *  \return True on success, false on failure.
             */
            bool addGroupToGroup(const std::string &group, const std::string &other);

            /** \} */

            /** \name Named States
                \{ */

            /** \brief Set the named states map.
             *  \param[in] states New named states map.
             */
            void setNamedGroupStates(const NamedStatesMap &states);

            /** \brief Get the named states map.
             *  \return The map of named states.
             */
            const NamedStatesMap &getNamedGroupStates() const;

            /** \brief Get all names for named states for a group.
             *  \param[in] group Group to get named states for.
             *  \return List of all named states.
             */
            std::vector<std::string> getNamedGroupStates(const std::string &group) const;

            /** \brief Get the configuration of a named group state.
             *  \param[in] group Group of the named state.
             *  \param[in] name Name of the state to get.
             *  \param[out] q Configuration to fill with named state.
             *  \return True if state exists, false if not.
             */
            bool getNamedGroupState(const std::string &group, const std::string &name,
                                    Eigen::Ref<Eigen::VectorXd> q) const;

            /** \brief Set or create a named group state.
             *  \param[in] group Group of the named state.
             *  \param[in] name Name of the state to set.
             *  \param[out] q Configuration for the named state.
             */
            void setNamedGroupState(const std::string &group, const std::string &name,
                                    const Eigen::Ref<const Eigen::VectorXd> &q);

            /** \} */

        private:
            /** \brief Adds a name to the group.
             *  \param[in] group Group to add name to.
             *  \param[in] name Name to add to group.
             *  \return True on success, false on failure.
             */
            bool addNameToGroup(const std::string &group, const std::string &name);

            /** \brief Process the members of a group to generate the set of indices.
             *  \param[in] group Group to process.
             */
            void processGroup(const std::string &group);

            std::map<std::string, std::vector<std::string>> groups_;  ///< Map of group names to joint names.
            std::map<std::string, std::map<std::string, Eigen::VectorXd>> group_states_;  ///< Map of group
                                                                                          ///< names to map of
                                                                                          ///< named states.
            std::map<std::string, std::vector<std::size_t>> group_indices_;  ///< Indices of group joints.

            robowflex::RobotPtr robot_{nullptr};  ///< Robowflex robot.
        };

        /** \brief Load a robot from a URDF and SRDF (i.e., a MoveIt enabled robot).
         *  \param[in] name Name of the robot.
         *  \param[in] urdf URDF URI.
         *  \param[in] srdf SRDF URI.
         *  \return The loaded robot.
         */
        RobotPtr loadMoveItRobot(const std::string &name, const std::string &urdf, const std::string &srdf);

    }  // namespace darts
}  // namespace robowflex

#endif
