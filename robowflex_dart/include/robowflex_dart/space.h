/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_SPACE_
#define ROBOWFLEX_DART_SPACE_

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <robowflex_library/class_forward.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/joints.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(StateSpace)
        /** \endcond */

        /** \class robowflex::darts::StateSpacePtr
            \brief A shared pointer wrapper for robowflex::darts::StateSpace. */

        /** \class robowflex::darts::StateSpaceConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::StateSpace. */

        /** \brief An OMPL state space for robowflex::darts::World. Can do motion planning for any of the
         * robots or structures in the world.
         */
        class StateSpace : public ompl::base::RealVectorStateSpace
        {
        public:
            /** \cond IGNORE */
            ROBOWFLEX_CLASS_FORWARD(StateSampler);
            /** \endcond */

            /** \class robowflex::darts::StateSpace::StateSamplerPtr
                \brief A shared pointer wrapper for robowflex::darts::StateSpace::StateSampler. */

            /** \class robowflex::darts::StateSpace::StateSamplerConstPtr
                \brief A const shared pointer wrapper for robowflex::darts::StateSpace::StateSampler. */

            /** \brief Sampler for robowflex::darts::StateSpace.
             */
            class StateSampler : public ompl::base::RealVectorStateSampler
            {
            public:
                /** \brief Constructor.
                 *  \param[in] space Space to sample.
                 */
                StateSampler(const StateSpace *space);

                /** \name OMPL StateSampler Methods
                    \{ */

                void sampleUniform(ompl::base::State *state) override;
                void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                       double distance) override;

                /** \} */

            private:
                const std::vector<JointPtr> &joints_;  ///< Joints to sample from.
            };

            /** \brief State type for the robowflex::darts::StateSpace.
             */
            class StateType : public ompl::base::RealVectorStateSpace::StateType
            {
            public:
                /** \brief Constructor.
                 *  \param[in] n Dimension of state.
                 */
                StateType(std::size_t n);

                Eigen::VectorXd data;  ///< Vector for configuration.
            };

            // Friendship
            friend Joint;
            friend RnJoint;
            friend StateSampler;

            /** \name Constructor and Setup
                \{ */

            /** \brief Constructor.
             *  \param[in] world World to plan for.
             */
            StateSpace(WorldPtr world);

            /** \brief Add a group to be planned for.
             *  \param[in] name Name of the robot that has the group.
             *  \param[in] group Group to plan for.
             *  \param[in] cyclic If >0, will flatten rotational groups (i.e.,
             *                    SO(2), SO(3)) into Rn spaces, where n is \a cyclic.
             */
            void addGroup(const std::string &name, const std::string &group, std::size_t cyclic = 0);

            /** \brief Add a group to be planned for.
             *  \param[in] group_name Name of the new group.
             *  \param[in] joints Joints to add to this group.
             *  \param[in] cyclic If >0, will flatten rotational groups (i.e.,
             *                    SO(2), SO(3)) into Rn spaces, where n is \a cyclic.
             */
            void addGroupFromJoints(const std::string &group_name,
                                    const std::vector<dart::dynamics::Joint *> &joints,
                                    std::size_t cyclic = 0);

            /** \} */

            /** \name World State
                \{ */

            /** \brief Set the state of a world from an OMPL state.
             *  \param[out] world World to set state of.
             *  \param[in] state State to set world.
             */
            void setWorldState(WorldPtr world, const ompl::base::State *state) const;

            /** \brief Set the state of a world from a configuration
             *  \param[out] world World to set state of.
             *  \param[in] x Configuration to set world.
             */
            void setWorldState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &x) const;

            /** \brief Get the state of a world in an OMPL state.
             *  \param[in] world World to get state of.
             *  \param[out] state State to fill.
             */
            void getWorldState(WorldPtr world, ompl::base::State *state) const;

            /** \brief Get the state of a world in a configuration vector.
             *  \param[in] world World to get state of.
             *  \param[out] x Configuration to fill.
             */
            void getWorldState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> x) const;

            /** \brief Set the state of a world from a configuration of a group.
             *  \param[out] world World to set state of.
             *  \param[in] group_name Name of group to set.
             *  \param[in] x Configuration to set.
             */
            void setWorldGroupState(WorldPtr world, const std::string &group_name,
                                    const Eigen::Ref<const Eigen::VectorXd> &x) const;

            /** \brief Get the group state of a world.
             *  \param[in] world World to get state of.
             *  \param[in] group_name Name of group to get.
             *  \param[out] x Configuration to get.
             */
            void getWorldGroupState(WorldPtr world, const std::string &group_name,
                                    Eigen::Ref<Eigen::VectorXd> x) const;

            /** \} */

            /** \name Getters and Setters
                \{ */

            /** \brief Get underlying world.
             *  \return The world for the space.
             */
            WorldPtr getWorld();

            /** \brief Get underlying world.
             *  \return The world for the space.
             */
            const WorldPtr &getWorldConst() const;

            /** \brief Get the set of indices that is being planned for. Pairs of skeleton index and joint
             * index.
             *  \return Vector of indices.
             */
            std::vector<std::pair<std::size_t, std::size_t>> getIndices() const;

            /** \brief Get a vector of the lower joint bounds for the space.
             *  \return The lower bounds of the space.
             */
            Eigen::VectorXd getLowerBound() const;

            /** \brief Get a vector of the upper joint bounds for the space.
             *  \return The upper bounds of the space.
             */
            Eigen::VectorXd getUpperBound() const;

            /** \brief Get a joint that is being planned for.
             *  \return The joint.
             */
            JointPtr getJoint(std::size_t index) const;

            /** \brief Get a joint that is being planned for.
             *  \return The joint.
             */
            JointPtr getJoint(const std::string &name) const;

            /** \brief Get a vector of the joints being planned for.
             *  \return The vector of joints being planned for.
             */
            const std::vector<JointPtr> &getJoints() const;

            /** \brief From a full state, get only the state of a group.
             *  \param[in] group The group to get the state for.
             *  \param[in] state The state to get the group state from.
             *  \param[out] v The group state to set.
             */
            void getGroupState(const std::string &group, const ompl::base::State *state,
                               Eigen::Ref<Eigen::VectorXd> v) const;

            /** \brief Set a (sub)group state in a full state.
             *  \param[in] group The group to set the state for.
             *  \param[out] state The state to set the group state in.
             *  \param[in] v The group state to use.
             */
            void setGroupState(const std::string &group, ompl::base::State *state,
                               const Eigen::Ref<const Eigen::VectorXd> &v) const;

            /** \brief Get the dimension of joint variables for a subgroup.
             *  \return The group's dimension.
             */
            std::size_t getGroupDimension(const std::string &group) const;

            /** \brief Get the names of all groups managed by this space.
             *  \return Names of all the groups.
             */
            std::vector<std::string> getGroups() const;

            /** \brief Get the names of each of the dof controlled by a group.
             *  \return The names of all the controlled dof for a group.
             */
            std::vector<std::string> getGroupDofNames(const std::string &group_name) const;

            /** \} */

            /** \name OMPL StateSpace Methods
                \{ */

            bool isMetricSpace() const override;
            void enforceBounds(ompl::base::State *state) const override;
            bool satisfiesBounds(const ompl::base::State *state) const override;
            double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;
            double getMaximumExtent() const override;
            bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;
            void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                             ompl::base::State *state) const override;
            ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;
            ompl::base::State *allocState() const override;
            void freeState(ompl::base::State *state) const override;

            /** \} */

            void setMetricSpace(bool metric);

        protected:
            void addJoint(const std::string &group_name, const JointPtr &joint);
            void addJointToGroup(const std::string &group_name, const JointPtr &joint);

            WorldPtr world_;  ///< World to use for planning.

            std::set<dart::dynamics::Joint *> jointset_;  ///< Set of joints used in planning.
            std::vector<std::size_t> indices_;            ///< Vector of indices for planning.

            bool metric_{true};  ///< Is this space all Rn?

            std::vector<JointPtr> joints_;  ///< Vector of all joints used in planning.

            ompl::RNG rng_;  ///< Random number generator.

            std::map<std::string, std::vector<JointPtr>> group_joints_;  ///< Joints belonging to a group.
            std::map<std::string, std::size_t> group_dimension_;         ///< Dimension of the group.
        };
    }  // namespace darts
}  // namespace robowflex

#endif
