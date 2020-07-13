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
             *  \param[in] cyclic If >0, will flatten rotational groups (i.e., SO(2), SO(3)) into Rn spaces.
             */
            void addGroup(const std::string &name, const std::string &group, std::size_t cyclic = 0);

            /** \} */

            /** \name World State
                \{ */

            /** \brief Set the state of a world from an OMPL state.
             *  \param[out] world World to set state of.
             *  \param[in] state State to set world.
             */
            void setWorldState(WorldPtr world, const ompl::base::State *state);

            /** \brief Set the state of a world from a configuration
             *  \param[out] world World to set state of.
             *  \param[in] x Configuration to set world.
             */
            void setWorldState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &x);

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
            WorldPtr world_;  ///< World to use for planning.

            /** \brief Set of groups used in planning. Tuple of robot name, group, and cyclic count.
             */
            std::vector<std::tuple<std::string, std::string, std::size_t>> groups_;

            std::set<dart::dynamics::Joint *> jointset_;  ///< Set of joints used in planning.
            std::vector<std::size_t> indices_;            ///< Vector of indices for planning.

            bool metric_{true};  ///< Is this space all Rn?

            std::vector<JointPtr> joints_;  ///< Vector of all joints used in planning.

            ompl::RNG rng_;  ///< Random number generator.
        };
    }  // namespace darts
}  // namespace robowflex

#endif
