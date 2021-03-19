/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_PLANNING_
#define ROBOWFLEX_DART_PLANNING_

#include <set>
#include <Eigen/Dense>

#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_dart/space.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(TSR)
        ROBOWFLEX_CLASS_FORWARD(TSRSet)
        ROBOWFLEX_CLASS_FORWARD(TSRConstraint)
        ROBOWFLEX_CLASS_FORWARD(World)
        /** \endcond */

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(TSRGoal)
        ROBOWFLEX_CLASS_FORWARD(JointRegionGoal)
        ROBOWFLEX_CLASS_FORWARD(PlanBuilder)
        /** \endcond */

        /** \class robowflex::darts::TSRGoalPtr
            \brief A shared pointer wrapper for robowflex::darts::TSRGoal. */

        /** \class robowflex::darts::TSRGoalConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::TSRGoal. */

        /** \class robowflex::darts::JointRegionGoalPtr
            \brief A shared pointer wrapper for robowflex::darts::JointRegionGoal. */

        /** \class robowflex::darts::JointRegionGoalConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::JointRegionGoal. */

        /** \brief Helper class to manage extracting states from a possibly constrained state space.
         */
        class ConstraintExtractor
        {
        public:
            /** \brief Constructor.
             */
            ConstraintExtractor() = default;

            /** \brief Constructor.
             *  \param[in] si The space information.
             */
            ConstraintExtractor(const ompl::base::SpaceInformationPtr &si);

            /** \name State Access
             *  \{ */

            /** \brief Extract underlying state from a base state.
             *  \param[in] state State.
             *  \return Underlying robot state.
             */
            StateSpace::StateType *toState(ompl::base::State *state) const;

            /** \brief Extract underlying state from a base state.
             *  \param[in] state State.
             *  \return Underlying robot state.
             */
            const StateSpace::StateType *toStateConst(const ompl::base::State *state) const;

            /** \brief Access the underlying state from a constrained OMPL state.
             *  \param[in] state Constrained state to access.
             *  \return The underlying state.
             */
            StateSpace::StateType *fromConstrainedState(ompl::base::State *state) const;

            /** \brief Access the underlying state from a constrained OMPL state.
             *  \param[in] state Constrained state to access.
             *  \return The underlying state.
             */
            const StateSpace::StateType *fromConstrainedStateConst(const ompl::base::State *state) const;

            /** \brief Access the underlying state from an unconstrained OMPL state.
             *  \param[in] state Unconstrained state to access.
             *  \return The underlying state.
             */
            StateSpace::StateType *fromUnconstrainedState(ompl::base::State *state) const;

            /** \brief Access the underlying state from an unconstrained OMPL state.
             *  \param[in] state Unconstrained state to access.
             *  \return The underlying state.
             */
            const StateSpace::StateType *fromUnconstrainedStateConst(const ompl::base::State *state) const;

            /** \} */

            /** \brief Gets the underlying state space from the space information.
             *  \return State space.
             */
            const StateSpace *getSpace() const;

            /** \brief Set space information used for constraint extraction.
             *  \param[in] si The space information.
             */
            void setSpaceInformation(const ompl::base::SpaceInformationPtr &si);

        private:
            ompl::base::SpaceInformationPtr space_info_;  ///< Space Information.
            bool is_constrained_{false};                  ///< Is the underlying space constrained?
        };

        /** \brief A sampleable goal region for OMPL for a set of TSRs.
         *  Samples goals in a separate thread using a clone of the world.
         */
        class TSRGoal : public ompl::base::GoalLazySamples, public ConstraintExtractor
        {
        public:
            /** \name Constructors.
                \{ */

            /** \brief Constructor.
             *  \param[in] si Space information.
             *  \param[in] world World to use.
             *  \param[in] tsrs TSRs to sample.
             */
            TSRGoal(const ompl::base::SpaceInformationPtr &si, const WorldPtr &world,
                    const std::vector<TSRPtr> &tsrs);

            /** \brief Constructor.
             *  \param[in] si Space information.
             *  \param[in] world World to use.
             *  \param[in] tsr TSR to sample.
             */
            TSRGoal(const ompl::base::SpaceInformationPtr &si, const WorldPtr &world, const TSRPtr tsr);

            /** \brief Constructor.
             *  \param[in] builder Plan builder to use as base for goal.
             *  \param[in] tsr TSR to sample.
             */
            TSRGoal(const PlanBuilder &builder, TSRPtr tsr);

            /** \brief Constructor.
             *  \param[in] builder Plan builder to use as base for goal.
             *  \param[in] tsrs TSRs to sample.
             */
            TSRGoal(const PlanBuilder &builder, const std::vector<TSRPtr> &tsrs);

            /** \brief Destructor. Stops sampling thread.
             */
            ~TSRGoal();

            /** \} */

            /** \brief Sampling routine. Generates IK samples from the TSR goal.
             *  \param[in] gls This class.
             *  \param[in] state State to sample.
             */
            bool sample(const ompl::base::GoalLazySamples *gls, ompl::base::State *state);

            /** \brief Distance to the goal.
             *  \param[in] state State to measure.
             */
            double distanceGoal(const ompl::base::State *state) const override;

            /** \brief Get the TSR set of this goal region.
             *  \return The TSR set.
             */
            TSRSetPtr getTSRSet();

            /** \brief Public options.
             */
            struct
            {
                bool use_gradient{false};     ///< Use gradient-based TSR solver.
                std::size_t max_samples{10};  ///< Maximum samples.
            } options;

        private:
            WorldPtr world_;                       ///< World used.
            TSRSetPtr tsr_;                        ///< TSR set to sample from.
            ompl::base::StateSamplerPtr sampler_;  ///< State sampler.
        };

        /** \brief A joint space goal volume.
         */
        class JointRegionGoal : public ompl::base::GoalSampleableRegion, public ConstraintExtractor
        {
        public:
            JointRegionGoal(const PlanBuilder &builder, const Eigen::Ref<const Eigen::VectorXd> &state,
                            double tolerance = 0.);

            JointRegionGoal(const PlanBuilder &builder, const Eigen::Ref<const Eigen::VectorXd> &lower,
                            const Eigen::Ref<const Eigen::VectorXd> &upper);

            ~JointRegionGoal();

            void sampleGoal(ompl::base::State *state) const override;
            unsigned int maxSampleCount() const override;
            double distanceGoal(const ompl::base::State *state) const override;

        private:
            mutable ompl::RNG rng_;

            ompl::base::State *lower_;
            ompl::base::State *upper_;
        };

        /** \class robowflex::darts::PlanBuilderPtr
            \brief A shared pointer wrapper for robowflex::darts::PlanBuilder. */

        /** \class robowflex::darts::PlanBuilderConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::PlanBuilder. */

        /** \brief A helper class to setup common OMPL structures for planning.
         */
        class PlanBuilder : public ConstraintExtractor
        {
        public:
            /** \name Setup and Initialization.
                \{ */

            /** \brief Constructor.
             *  \param[in] world World to use for planning.
             */
            PlanBuilder(WorldPtr world);

            /** \brief Initialize OMPL structures. Only call this once all constraints, groups, and other
             * parameters have been set for the builder.
             */
            void initialize();

            /** \brief Calls setup routines on OMPL structures. Call after all setup is done and builder is
             * initialized.
             */
            void setup();

            /** \} */

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

            /** \name Other Settings
                \{ */

            /** \brief Add a robot's planning group to the controlled DoF in the state space.
             *  \param[in] robot Name of the robot.
             *  \param[in] name Name of the group in the robot to add.
             *  \param[in] cyclic If > 0, if the group contains cyclic joints (SO(2), SO(3)) these will be
             * modeled as Rn.
             */
            void addGroup(const std::string &robot, const std::string &name, std::size_t cyclic = 0);

            /** \brief Adds a TSR as a path constraint to the problem.
             *  \param[in] tsr Path constraint.
             */
            void addConstraint(const TSRPtr &tsr);

            /** \} */

            /** \name Start Configuration
                \{ */

            /** \brief Set the start configuration from the current state of the world.
             */
            void setStartConfigurationFromWorld();

            /** \brief Set the start configuration from a configuration.
             *  \param[in] q Configuration.
             */
            void setStartConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q);

            /** \brief Set the start configuration from a configuration.
             *  \param[in] q Configuration.
             */
            void setStartConfiguration(const std::vector<double> &q);

            /** \brief Sample a valid start configuration.
             */
            void sampleStartConfiguration();

            /** \} */

            /** \name Goals
                \{ */

            /** \brief Set the goal configuration from the current state of the world.
             */
            std::shared_ptr<ompl::base::GoalStates> getGoalConfigurationFromWorld();

            /** \brief Set the goal configuration from a configuration.
             *  \param[in] q Configuration.
             */
            std::shared_ptr<ompl::base::GoalStates>
            getGoalConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q);

            /** \brief Set the goal configuration from a configuration.
             *  \param[in] q Configuration.
             */
            std::shared_ptr<ompl::base::GoalStates> getGoalConfiguration(const std::vector<double> &q);

            /** \brief Set a TSR as the goal for the planning problem (will create a TSRGoal)
             *  \param[in] tsr TSR for the goal.
             */
            TSRGoalPtr getGoalTSR(const TSRPtr &tsr);

            /** \brief Set a set of TSRs as the goal for the planning problem (will create a TSRGoal)
             *  \param[in] tsrs TSRs for the goal.
             */
            TSRGoalPtr getGoalTSR(const std::vector<TSRPtr> &tsrs);

            /** \brief Sample a valid goal configuration.
             */
            std::shared_ptr<ompl::base::GoalStates> sampleGoalConfiguration();

            /** \brief Set the goal for the problem.
             *  \param[in] goal Goal to use.
             */
            void setGoal(const ompl::base::GoalPtr &goal);

            /** \} */

            /** \name Other Functions
                \{ */

            /** \brief Sample a valid state.
             *  \return a Valid state.
             */
            ompl::base::State *sampleState() const;

            /** \brief Get the solution path from a successful plan
             *  \param[in] simplify Simplify the solution.
             *  \param[in] interpolate Interpolate the solution.
             */
            ompl::geometric::PathGeometric getSolutionPath(bool simplify = true,
                                                           bool interpolate = true) const;

            /** \} */

            StateSpacePtr rspace{nullptr};             ///< Underlying Robot State Space.
            ompl::base::StateSpacePtr space{nullptr};  ///< Actual OMPL State Space (might be constrained).
            ompl::base::SpaceInformationPtr rinfo{nullptr};  ///< Underlying Space Information.
            ompl::base::SpaceInformationPtr info{nullptr};   ///< Actual Space Information.
            ompl::geometric::SimpleSetupPtr ss{nullptr};     ///< Simple Setup.
            WorldPtr world;                                  ///< World used for planning.

            std::vector<TSRPtr> path_constraints;  ///< Path Constraints.
            TSRConstraintPtr constraint{nullptr};  ///< OMPL Constraint for Path Constraints.

            Eigen::VectorXd start;  ///< Start configuration.

            /** \brief Hyperparameter options.
             */
            struct
            {
                /** \brief Constrained planning options.
                 */
                struct
                {
                    double delta{0.2};   ///< Manifold delta.
                    double lambda{5.0};  ///< Manifold lambda.
                } constraints;
            } options;

        protected:
            ompl::base::GoalPtr goal_{nullptr};  ///< Desired goal.

            /** \brief Initialize when path constraints are present.
             */
            void initializeConstrained();

            /** \brief Initialize when no path constraints are present.
             */
            void initializeUnconstrained();

            /** \brief Set the state validity checker on the simple setup.
             */
            void setStateValidityChecker();

            /** \brief Get a state validity checker for the unconstrained space.
             *  \return The state validity checker.
             */
            ompl::base::StateValidityCheckerFn getSVCUnconstrained();

            /** \brief Get a state validity checker for the unconstrained space.
             *  \return The state validity checker.
             */
            ompl::base::StateValidityCheckerFn getSVCConstrained();
        };
    }  // namespace darts
}  // namespace robowflex

#endif
