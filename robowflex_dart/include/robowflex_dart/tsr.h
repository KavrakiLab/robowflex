/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_TSR_
#define ROBOWFLEX_DART_TSR_

#include <set>
#include <functional>
#include <iostream>

#include <ompl/base/Constraint.h>

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(World)
        ROBOWFLEX_CLASS_FORWARD(Structure)
        ROBOWFLEX_CLASS_FORWARD(StateSpace)
        /** \endcond */

        namespace magic
        {
            static const double DEFAULT_IK_TOLERANCE = 1e-3;
            static const std::string ROOT_FRAME = "";
            static const Eigen::Vector3d DEFAULT_IK_TOLERANCES =
                Eigen::Vector3d::Constant(DEFAULT_IK_TOLERANCE);
        }  // namespace magic

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(TSR)
        /** \endcond */

        /** \class robowflex::darts::TSRPtr
            \brief A shared pointer wrapper for robowflex::darts::TSR. */

        /** \class robowflex::darts::TSRConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::TSR. */

        /** \brief A Task Space Region (TSR). TSRs are workspace regions that impose a constraints on a robot.
         * They are composed of a pose (in some reference frame) and tolerances on position and rotation.
         * initialize() must be called once all parameters are set.
         *
         * In the TSR, there are controlled DoF (through useIndices()) and world DoF (through
         * useWorldIndices()). The controlled DoF are the DoF inside the structure the TSR's target frame is.
         * These are the DoF that are used as part of the error, Jacobian, and other computations. However,
         * the world might be using more DoF, especially if it is a multi-robot system. The world's DoF are
         * given through useWorldIndices(). The *WorldState allow you to pass a configuration for the entire
         * world (where each entry in that configuration corresponds to the indices given) and have the input
         * / output be mapped appropriately to the controlled DoF.
         *
         * Note, the TSR creates an IK node in the underlying Dart skeleton for the provided world. If you
         * have multiple TSRs for the same target frame these will overwrite each other, unless you use a
         * clone of the World.
         */
        class TSR
        {
            ROBOWFLEX_EIGEN

        public:
            /** \brief The specification of a TSR.
             */
            class Specification
            {
            public:
                ROBOWFLEX_EIGEN

                struct
                {
                    std::string structure;  ///< Structure target frame is in.
                    std::string frame;      ///< Name of target frame.
                } target;                   ///< Target frame.

                struct
                {
                    std::string structure;                 ///< Structure base frame is in.
                    std::string frame{magic::ROOT_FRAME};  ///< Name of base frame.
                } base;                                    ///< Base frame.

                RobotPose pose{RobotPose::Identity()};  ///< Pose of TSR.

                struct
                {
                    ROBOWFLEX_EIGEN
                    Eigen::Vector3d upper{magic::DEFAULT_IK_TOLERANCES};   ///< Upper position tolerance.
                    Eigen::Vector3d lower{-magic::DEFAULT_IK_TOLERANCES};  ///< Lower position tolerance.
                } position;                                                ///< Position tolerances.

                struct
                {
                    ROBOWFLEX_EIGEN
                    Eigen::Vector3d upper{magic::DEFAULT_IK_TOLERANCES};   ///< Upper orientation tolerance.
                    Eigen::Vector3d lower{-magic::DEFAULT_IK_TOLERANCES};  ///< Lower orientation tolerance.
                } orientation;                                             ///< Orientation tolerances.

                std::size_t dimension{6};  ///< Number of constrained dimensions.

                /** \brief Vector of active constraints. If an index is true, that constraint is active.
                 * Ordered by X-, Y-, Z- orientation constraints, then X-, Y-, Z- position.
                 */
                std::vector<bool> indices{std::vector<bool>(6, true)};

                std::size_t maxIter{50};                        ///< Maximum iterations for solver.
                double tolerance{magic::DEFAULT_IK_TOLERANCE};  ///< Tolerance on solution for solver.

                /** \name Constructors
                    \{ */

                /** \brief Default constructor.
                 */
                Specification() = default;

                /** \brief Constructor for basic pose TSR constrained in world frame.
                 *  \param[in] structure Structure TSR is on (both target and base frames).
                 *  \param[in] target_frame Target frame.
                 *  \param[in] position Desired position.
                 *  \param[in] rotation Desired orientation.
                 */
                Specification(const std::string &structure, const std::string &target_frame,
                              const Eigen::Ref<const Eigen::Vector3d> &position,
                              const Eigen::Quaterniond &rotation);

                /** \} */

                /** \brief Setting TSR Frame
                    \{ */

                /** \brief Set the target frame. Sets base frame structure as well if not already set.
                 *  \param[in] structure Structure frame is in.
                 *  \param[in] frame Frame name.
                 */
                void setTarget(const std::string &structure, const std::string &frame);

                /** \brief Set the base frame.
                 *  \param[in] structure Structure frame is in.
                 *  \param[in] frame Frame name.
                 */
                void setBase(const std::string &structure, const std::string &frame);

                /** \brief Set the base and target frame.
                 *  \param[in] structure Structure frames are in.
                 *  \param[in] target Target frame name.
                 *  \param[in] base Base frame name.
                 */
                void setFrame(const std::string &structure, const std::string &target,
                              const std::string &base = magic::ROOT_FRAME);

                /** \brief Add a suffix to the structures for the target and base frame.
                 *  \param[in] suffix Suffix to add to structures in target and base frame.
                 */
                void addSuffix(const std::string &suffix);

                /** \} */

                /** \brief Setting TSR Pose
                    \{ */

                /** \brief Set the position of the TSR.
                 *  \param[in] position Position vector.
                 */
                void setPosition(const Eigen::Ref<const Eigen::Vector3d> &position);

                /** \brief Set the position of the TSR.
                 *  \param[in] x X-coordinate.
                 *  \param[in] y Y-coordinate.
                 *  \param[in] z Z-coordinate.
                 */
                void setPosition(double x, double y, double z);

                /** \brief Set the rotation of the TSR.
                 *  \param[in] orientation Desired quaternion.
                 */
                void setRotation(const Eigen::Quaterniond &orientation);

                /** \brief Set the rotation of the TSR as a quaternion.
                 *  \param[in] w W-component of quaternion.
                 *  \param[in] x X-component of quaternion.
                 *  \param[in] y Y-component of quaternion.
                 *  \param[in] z Z-component of quaternion.
                 */
                void setRotation(double w, double x, double y, double z);

                /** \brief Set the rotation of the TSR as XYZ Euler angles.
                 *  \param[in] x X-component of rotation.
                 *  \param[in] y Y-component of rotation.
                 *  \param[in] z Z-component of rotation.
                 */
                void setRotation(double x, double y, double z);

                /** \brief Set the pose of the TSR.
                 *  \param[in] other Pose to use.
                 */
                void setPose(const RobotPose &other);

                /** \brief Set the pose of the TSR.
                 *  \param[in] position Desired position.
                 *  \param[in] rotation Desired rotation.
                 */
                void setPose(const Eigen::Ref<const Eigen::Vector3d> &position,
                             const Eigen::Quaterniond &rotation);

                /** \brief Set the pose of the TSR.
                 *  \param[in] xp X-coordinate of position.
                 *  \param[in] yp Y-coordinate of position.
                 *  \param[in] zp Z-coordinate of position.
                 *  \param[in] wr W-component of quaternion.
                 *  \param[in] xr X-component of quaternion.
                 *  \param[in] yr Y-component of quaternion.
                 *  \param[in] zr Z-component of quaternion.
                 */
                void setPose(double xp, double yp, double zp, double wr, double xr, double yr, double zr);

                /** \brief Set the pose of the TSR for the desired frame in a provided world. Uses world's
                 * current configuration. \param[in] world World to copy transform from.
                 */
                void setPoseFromWorld(const WorldPtr &world);

                /** \} */

                /** \brief Setting Position Tolerances
                    \{ */

                /** \brief Set the X position tolerance to (-bound, bound).
                 *  \param[in] bound Bound to set.
                 */
                void setXPosTolerance(double bound);

                /** \brief Set the Y position tolerance to (-bound, bound).
                 *  \param[in] bound Bound to set.
                 */
                void setYPosTolerance(double bound);

                /** \brief Set the Z position tolerance to (-bound, bound).
                 *  \param[in] bound Bound to set.
                 */
                void setZPosTolerance(double bound);

                /** \brief Set the X position tolerance to (lower, upper).
                 *  \param[in] lower Lower bound.
                 *  \param[in] upper Upper bound.
                 */
                void setXPosTolerance(double lower, double upper);

                /** \brief Set the Y position tolerance to (lower, upper).
                 *  \param[in] lower Lower bound.
                 *  \param[in] upper Upper bound.
                 */
                void setYPosTolerance(double lower, double upper);

                /** \brief Set the Z position tolerance to (lower, upper).
                 *  \param[in] lower Lower bound.
                 *  \param[in] upper Upper bound.
                 */
                void setZPosTolerance(double lower, double upper);

                /** \brief Set no position tolerance on the X-axis.
                 */
                void setNoXPosTolerance();

                /** \brief Set no position tolerance on the Y-axis.
                 */
                void setNoYPosTolerance();

                /** \brief Set no position tolerance on the Z-axis.
                 */
                void setNoZPosTolerance();

                /** \brief Set no position tolerance at all.
                 */
                void setNoPosTolerance();

                /** \} */

                /** \brief Setting Orientation Tolerances
                    \{ */

                /** \brief Set the X orientation tolerance to (-bound, bound).
                 *  \param[in] bound Bound to set.
                 */
                void setXRotTolerance(double bound);

                /** \brief Set the Y orientation tolerance to (-bound, bound).
                 *  \param[in] bound Bound to set.
                 */
                void setYRotTolerance(double bound);

                /** \brief Set the Z orientation tolerance to (-bound, bound).
                 *  \param[in] bound Bound to set.
                 */
                void setZRotTolerance(double bound);

                /** \brief Set the X orientation tolerance to (lower, upper).
                 *  \param[in] lower Lower bound.
                 *  \param[in] upper Upper bound.
                 */
                void setXRotTolerance(double lower, double upper);

                /** \brief Set the Y orientation tolerance to (lower, upper).
                 *  \param[in] lower Lower bound.
                 *  \param[in] upper Upper bound.
                 */
                void setYRotTolerance(double lower, double upper);

                /** \brief Set the Z orientation tolerance to (lower, upper).
                 *  \param[in] lower Lower bound.
                 *  \param[in] upper Upper bound.
                 */
                void setZRotTolerance(double lower, double upper);

                /** \brief Set no orientation tolerance on the X-axis.
                 */
                void setNoXRotTolerance();

                /** \brief Set no orientation tolerance on the Y-axis.
                 */
                void setNoYRotTolerance();

                /** \brief Set no orientation tolerance on the Z-axis.
                 */
                void setNoZRotTolerance();

                /** \brief Set no orientation tolerance at all.
                 */
                void setNoRotTolerance();

                /** \} */

                /** \brief Getters and Informative Methods
                    \{ */

                /** \brief Get the current desired position.
                 *  \return Position vector.
                 */
                Eigen::Vector3d getPosition() const;

                /** \brief Get the current desired orientation.
                 *  \return Orientation quaternion.
                 */
                Eigen::Quaterniond getRotation() const;

                /** \brief Get the current desired orientation.
                 *  \return Orientation as XYZ Euler angles.
                 */
                Eigen::Vector3d getEulerRotation() const;

                /** \brief Returns true if TSR is position constrained.
                 *  \return True if position constrained.
                 */
                bool isPositionConstrained() const;

                /** \brief Returns true if TSR is orientation constrained.
                 *  \return True if orientation constrained.
                 */
                bool isRotationConstrained() const;

                /** \brief Returns true if TSR is a relative reference frame (not the world).
                 *  \return True if this is a relative reference frame.
                 */
                bool isRelative() const;

                /** \brief Print out this TSR information.
                 *  \param[in] out Output stream.
                 */
                void print(std::ostream &out) const;

                /** \} */

                /** \brief Operations
                    \{ */

                /** \brief Compute the intersection of this TSR with the \a other.
                 *  \return True if successful, false on failure.
                 */
                bool intersect(const Specification &other);

                /** \} */

            private:
                /** \brief Fixes bounds so they are correct.
                 */
                void fixBounds();

                /** \brief Compute and return constraint dimension of the TSR.
                 *  \return Dimension of TSR.
                 */
                std::size_t getDimension() const;

                /** \brief Checks if two values correspond to a position constraint.
                 *  \return If (lower, upper) is bounded or not.
                 */
                bool isPosConstrained(double lower, double upper) const;

                /** \brief Checks if two values correspond to a orientation constraint.
                 *  \return If (lower, upper) is bounded or not.
                 */
                bool isRotConstrained(double lower, double upper) const;
            };

            /** \name Constructor and Initialization
                \{ */

            /** \brief Constructor.
             *  \param[in] world World to apply TSR to.
             *  \param[in] spec Specification for TSR.
             */
            TSR(const WorldPtr &world, const Specification &spec);

            /** \brief Destructor. Disables IK on node if created.
             */
            ~TSR();

            /** \brief Set the world used by this TSR.
             *  \param[in] world New world to use.
             */
            void setWorld(const WorldPtr &world);

            /** \brief Clears the initialization of this TSR.
             */
            void clear();

            /** \brief Initialize this TSR. Creates IK node attached to body node in world.
             */
            void initialize();

            /** \} */

            /** \name DoF Indexing
                \{ */

            /** \brief Use the joints in a robot's group. Robot used is the target frame's structure.
             *  \param[in] name Name of group to use.
             */
            void useGroup(const std::string &name);

            /** \brief Use DoF indices for TSR computation.
             *  \param[in] indices Indices to use.
             */
            void useIndices(const std::vector<std::size_t> &indices);

            /** \brief Gets the transformation from the specification's base to the TSR's frame.
             * \return RobotPose representing the transform.
             */
            robowflex::RobotPose getTransformToFrame() const;

            /** \brief Use World DoF indices for TSR computation. World indices are pairs of skeleton index
             * and DoF index.
             *  \param[in] indices Indices to use.
             */
            void useWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices);

            /** \brief Output world indices. TSR information for *WorldState methods uses this set
             * of world indices. World indices are pairs of skeleton index and DoF index.
             *  \param[in] indices Indices to use.
             */
            void setWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices);

            /** \brief Compute the set of world indices used based on the underlying active indices.
             *  \return World indices corresponding to this TSR's current indices.
             */
            std::vector<std::pair<std::size_t, std::size_t>> computeWorldIndices() const;

            /** \brief Get the skeleton index for the target frame's structure.
             *  \return The index of the skeleton this TSR is targeting.
             */
            std::size_t getSkeletonIndex();

            /** \brief Get the set of indices used for TSR computation.
             */
            const std::vector<std::size_t> &getIndices() const;

            /** \brief Get the set of output world indices used.
             */
            const std::vector<std::pair<std::size_t, std::size_t>> &getWorldIndices() const;

            /** \brief From a configuration of controlled indices, map that configuration into a world
             * configuration. That is, map \a state into \a world according to the set indices and world
             * indices.
             *  \param[out] world Output world configuration.
             *  \param[in] state Input configuration.
             */
            void toBijection(Eigen::Ref<Eigen::VectorXd> world,
                             const Eigen::Ref<const Eigen::VectorXd> &state) const;

            /** \brief From a world configuration, map that configuration into a controlled DoF
             * configuration. That is, map \a world into \a state according to the set indices and world
             * indices.
             *  \param[out] state Output configuration.
             *  \param[in] world Input world configuration.
             */
            void fromBijection(Eigen::Ref<Eigen::VectorXd> state,
                               const Eigen::Ref<const Eigen::VectorXd> &world) const;

            /** \} */

            /** \name Getters
                \{ */

            /** \brief Get the error dimension of this TSR.
             *  \return The error dimension of the TSR.
             */
            std::size_t getDimension() const;

            /** \brief Get the number of controlled DoF for this TSR.
             *  \return The number of DoF for this TSR.
             */
            std::size_t getNumDofs() const;

            /** \brief Get the number of world DoF for this TSR.
             *  \return The number of world DoF for this TSR.
             */
            std::size_t getNumWorldDofs() const;

            /** \} */

            /** \name Specification and Updates
                \{ */

            /** \brief Get the specification for this TSR.
             *  \return The TSR's specification.
             */
            Specification &getSpecification();

            /** \brief If the pose of the specification is updated, call this to update underlying IK solver.
             */
            void updatePose();

            /** \brief If the bounds of the specification are updated, call this to update underlying IK
             * solver.
             */
            void updateBounds();

            /** \brief If the solver parameters of the specification are updated, call this to update
             * underlying IK solver.
             */
            void updateSolver();

            /** \} */

            /** \name Error / Function Computation
                \{ */

            /** \brief Get the current error given the world state, with all values.
             *  \param[out] error Error value for all TSRs.
             */
            void getErrorWorldRaw(Eigen::Ref<Eigen::VectorXd> error) const;

            /** \brief Get the current error given the world state.
             *  \param[out] error Error value.
             */
            void getErrorWorld(Eigen::Ref<Eigen::VectorXd> error) const;

            /** \brief Get the error given a provided world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \param[out] error Error value.
             */
            void getErrorWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                    Eigen::Ref<Eigen::VectorXd> error) const;

            /** \brief Get the error given a configuration of the controlled DoF.
             *  \param[in] state Controlled DoF configuration.
             *  \param[out] error Error value.
             */
            void getError(const Eigen::Ref<const Eigen::VectorXd> &state,
                          Eigen::Ref<Eigen::VectorXd> error) const;

            /** \} */

            /** \name Jacobian Computation
                \{ */

            /** \brief Get the current Jacobian given the world state.
             *  \param[out] jacobian Jacobian value.
             */
            void getJacobianWorld(Eigen::Ref<Eigen::MatrixXd> jacobian) const;

            /** \brief Get the Jacobian given a provided world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \param[out] jacobian Jacobian value.
             */
            void getJacobianWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                       Eigen::Ref<Eigen::MatrixXd> jacobian) const;

            /** \brief Get the Jacobian given a configuration of the controlled DoF.
             *  \param[in] state Controlled DoF configuration.
             *  \param[out] jacobian Jacobian value.
             */
            void getJacobian(const Eigen::Ref<const Eigen::VectorXd> &state,
                             Eigen::Ref<Eigen::MatrixXd> jacobian) const;

            /** \} */

            /** \name Distance Computation
                \{ */

            /** \brief Get the current distance to satisfaction given the world state.
             *  \return Distance to TSR satisfaction.
             */
            double distanceWorld() const;

            /** \brief Get the distance to satisfaction given a provided world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \return Distance to TSR satisfaction.
             */
            double distanceWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const;

            /** \brief Get the distance to satisfaction given a configuration of the controlled DoF.
             *  \param[in] state Controlled DoF configuration.
             *  \return Distance to TSR satisfaction.
             */
            double distance(const Eigen::Ref<const Eigen::VectorXd> &state) const;

            /** \} */

            /** \name Solving for Satisfying Configurations
                \{ */

            /** \brief Solve for a satisfying configuration given the current state of the world.
             *  \return True on success, false on failure.
             */
            bool solveWorld();

            /** \brief Solve for a satisfying configuration given an initial world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \return True on success, false on failure.
             */
            bool solveWorldState(Eigen::Ref<Eigen::VectorXd> world);

            /** \brief Solve for a satisfying configuration given an initial configuration of the controlled
             * DoF.
             *  \param[in] state Controlled DoF configuration.
             *  \return True on success, false on failure.
             */
            bool solve(Eigen::Ref<Eigen::VectorXd> state);

            /** \brief Solve using gradient descent for a satisfying configuration given the current state of
             * the world. \return True on success, false on failure.
             */
            bool solveGradientWorld();

            /** \brief Solve using gradient descent for a satisfying configuration given an initial world
             * configuration.
             *  \param[in] world World configuration (from world indices).
             *  \return True on success, false on failure.
             */
            bool solveGradientWorldState(Eigen::Ref<Eigen::VectorXd> world);

            /** \brief Solve using gradient descent for a satisfying configuration given an initial
             * configuration of the controlled DoF.
             *  \param[in] state Controlled DoF configuration.
             *  \return True on success, false on failure.
             */
            bool solveGradient(Eigen::Ref<Eigen::VectorXd> state);

            /** \} */

            /** \name Getting and Setting Position
                \{ */

            /** \brief Set the positions of the world given a world configuration.
             *  \param[in] world World configuration.
             */
            void setPositionsWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const;

            /** \brief Set the positions of the world given a configuration.
             *  \param[in] state Controlled DoF configuration.
             */
            void setPositions(const Eigen::Ref<const Eigen::VectorXd> &state) const;

            /** \brief Get the positions of the world into a world configuration.
             *  \param[out] world World configuration.
             */
            void getPositionsWorldState(Eigen::Ref<Eigen::VectorXd> world) const;

            /** \brief Get the positions of the world given into a configuration.
             *  \param[out] state Controlled DoF configuration.
             */
            void getPositions(Eigen::Ref<Eigen::VectorXd> state) const;

            /** \} */

        private:
            /** \brief Compute the mapping from world indices to controlled DoF indices.
             */
            void computeBijection();

            WorldPtr world_;      ///< Underlying world.
            Specification spec_;  ///< TSR specification.

            std::size_t skel_index_;            ///< Index of controlled skeleton.
            std::vector<std::size_t> indices_;  ///< Controlled indices.
            std::vector<std::pair<std::size_t, std::size_t>> world_indices_;  ///< World indices.
            std::vector<std::size_t> bijection_;  ///< Mapping between controlled to world indices. The ith
                                                  ///< entry contains the mapping of state[i] =
                                                  ///< world[bijection_[i]]

            std::shared_ptr<dart::dynamics::SimpleFrame> frame_{nullptr};     ///< Target frame of TSR.
            dart::dynamics::BodyNode *tnd_{nullptr};                          ///< Target body node.
            std::shared_ptr<dart::dynamics::InverseKinematics> ik_{nullptr};  ///< Inverse kinematics module.
            dart::dynamics::InverseKinematics::TaskSpaceRegion *tsr_{nullptr};  ///< TSR.
        };

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(TSRSet)
        /** \endcond */

        /** \class robowflex::darts::TSRSetPtr
            \brief A shared pointer wrapper for robowflex::darts::TSRSet. */

        /** \class robowflex::darts::TSRSetConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::TSRSet. */

        /** \brief Manager for a set of TSR constraints. Attempts to reduce redundancy and combines errors and
         * Jacobians together.
         *
         *  Will "stack" the error values and Jacobians together. For example, given two TSRs \a "a" and \a
         * "b" in the set, the error vector will be [ea, eb] where \a ea and \a eb are \a a and \a b's error
         * vector. The Jacobian will also correspond to this stacked error vector.
         */
        class TSRSet
        {
        public:
            /** \name Constructor and Initialization
                \{ */

            /** \brief Constructor.
             *  \param[in] world World to use.
             *  \param[in] tsr TSR to add to set.
             */
            TSRSet(const WorldPtr &world, const TSRPtr &tsr);

            /** \brief Constructor.
             *  \param[in] world World to use.
             *  \param[in] tsrs TSRs to add to set.
             *  \param[in] intersect If true, tries to simplify TSR set.
             */
            TSRSet(const WorldPtr &world, const std::vector<TSRPtr> &tsrs, bool intersect = true);

            /** \brief Set the world used by this TSR set.
             *  \param[in] world New world to use.
             */
            void setWorld(const WorldPtr &world);

            /** \brief Add a TSR to the set.
             *  \param[in] tsr TSR to add.
             *  \param[in] intersect If true, tries to simplify TSR set.
             *  \param[in] weight Weight to use for this TSR.
             */
            void addTSR(const TSRPtr &tsr, bool intersect = true, double weight = 1.0);

            /** \brief Initialize this set of TSRs
             */
            void initialize();

            /** \brief Update the solver information (iterations, tolerance, etc.)
             */
            void updateSolver();

            /** \} */

            /** \name DoF Indexing
                \{ */

            /** \brief Use the joints in a robot's group. Robot used is the target frame's structure.
             *  \param[in] name Name of group to use.
             */
            void useGroup(const std::string &name);

            /** \brief Use DoF indices for all TSRs computation.
             *  \param[in] indices Indices to use.
             */
            void useIndices(const std::vector<std::size_t> &indices);

            /** \brief Use World DoF indices for all TSRs computation. World indices are pairs of skeleton
             * index and DoF index. \param[in] indices Indices to use.
             */
            void useWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices);

            /** \brief Output world indices. TSR information for *WorldState methods uses this set
             * of world indices. World indices are pairs of skeleton index and DoF index.
             *  \param[in] indices Indices to use.
             */
            void setWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices);

            /** \brief Get the world indices used.
             *  \return World indices.
             */
            const std::vector<std::pair<std::size_t, std::size_t>> &getWorldIndices() const;

            /** \brief Set the upper limit on joints given their index in the world configuration.
             *  \param[in] upper Upper bounds on joints.
             */
            void setWorldUpperLimits(const Eigen::Ref<const Eigen::VectorXd> &upper);

            /** \brief Set the lower limit on joints given their index in the world configuration.
             *  \param[in] lower Lower bounds on joints.
             */
            void setWorldLowerLimits(const Eigen::Ref<const Eigen::VectorXd> &lower);

            /** \brief Compute the upper and lower limits from the skeleton.
             */
            void computeLimits();

            /** \} */

            /** \name Getters
                \{ */

            /** \brief Get the current world state.
             *  \param[out] world State to fill.
             */
            void getPositionsWorldState(Eigen::Ref<Eigen::VectorXd> world) const;

            /** \brief Get the error dimension of this set of TSRs.
             *  \return The error dimension of the set of TSRs.
             */
            std::size_t getDimension() const;

            /** \brief Get the number of TSRs in the set.
             *  \return The number of TSRs.
             */
            std::size_t numTSRs() const;

            /** \brief Get the TSRs that form the set.
             *  \return The set of TSRs.
             */
            const std::vector<TSRPtr> &getTSRs() const;

            /** \brief Get the numerical tolerance for solving.
             *  \return The tolerance.
             */
            double getTolerance() const;

            /** \brief Set the maximum tolerance used for solving.
             *  \param[in] tolerance New tolerance.
             */
            void setTolerance(double tolerance);

            /** \brief Get the maximum iterations allowed.
             *  \return The maximum solver iterations.
             */
            std::size_t getMaxIterations() const;

            /** \brief Set the maximum iterations used for solving.
             *  \param[in] iterations Max iterations to use.
             */
            void setMaxIterations(std::size_t iterations);

            /** \brief Add a suffix to the end of all structures in the TSRs in the set.
             *  \param[in] suffix Suffix to add.
             */
            void addSuffix(const std::string &suffix);

            /** \brief Set the step parameter in the solver.
             *  \param[in] step Step parameter.
             */
            void setStep(double step);

            /** \brief Get the solver step parameter.
             *  \return The solver step parameter.
             */
            double getStep() const;

            /** \brief Set the limit parameter in the solver.
             *  \param[in] limit Limit parameter.
             */
            void setLimit(double limit);

            /** \brief Get the solver limit parameter.
             *  \return The solver limit parameter.
             */
            double getLimit() const;

            /** \brief Set the damping parameter in the solver.
             *  \param[in] damping Damping parameter.
             */
            void setDamping(double damping);

            /** \brief Get the solver damping parameter.
             *  \return The solver damping parameter.
             */
            double getDamping() const;

            /** \brief Set if damping is used in SVD solving.
             *  \param[in] damping Damping value.
             */
            void useDamping(bool damping);

            /** \brief Use SVD for solving.
             */
            void useSVD();

            /** \brief Use QR for solving.
             */
            void useQR();

            /** \} */

            /** \name Error / Function Computation
                \{ */

            /** \brief Get the current error given the world state.
             *  \param[out] error Error value for all TSRs.
             */
            void getErrorWorld(Eigen::Ref<Eigen::VectorXd> error) const;

            /** \brief Get the error given a provided world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \param[out] error Error value for all TSRs.
             */
            void getErrorWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                    Eigen::Ref<Eigen::VectorXd> error) const;

            /** \} */

            /** \name Jacobian Computation
                \{ */

            /** \brief Get the Jacobian given a provided world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \param[out] jacobian Jacobian value for all TSRs.
             */
            void getJacobianWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                       Eigen::Ref<Eigen::MatrixXd> jacobian) const;

            /** \} */

            /** \name Distance Computation
                \{ */

            /** \brief Get the current distance to satisfaction given the world state.
             *  \return Distance to TSR set satisfaction.
             */
            double distanceWorld() const;

            /** \brief Get the distance to satisfaction given a provided world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \return Distance to TSR set satisfaction.
             */
            double distanceWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const;

            /** \} */

            /** \name Solving for Satisfying Configurations
                \{ */

            /** \brief Solve for a satisfying configuration given the current state of the world.
             *  \return True on success, false on failure.
             */
            bool solveWorld();

            /** \brief Solve for a satisfying configuration given an initial world configuration.
             *  \param[in] world World configuration (from world indices).
             *  \return True on success, false on failure.
             */
            bool solveWorldState(Eigen::Ref<Eigen::VectorXd> world);

            /** \brief Solve using gradient descent for a satisfying configuration given an initial world
             * configuration.
             *  \param[in] world World configuration (from world indices).
             *  \return True on success, false on failure.
             */
            bool solveGradientWorldState(Eigen::Ref<Eigen::VectorXd> world);

            /** \} */

            /** \brief Print out the TSRs in this set.
             *  \param[in] out Output stream.
             */
            void print(std::ostream &out) const;

        private:
            /** \brief Enforce upper and lower bounds on a world configuration.
             *  \param[in,out] world World configuration to enforce.
             */
            void enforceBoundsWorld(Eigen::Ref<Eigen::VectorXd> world) const;

            WorldPtr world_;                      ///< World to use.
            std::set<std::size_t> skel_indices_;  ///< All skeleton indices used by members of the set.

            bool qr_{false};        ///< If true, use QR in gradient solve. Else, SVD.
            bool damped_{true};     ///< If true, use damped SVD.
            double step_{1.0};      ///< Step scaling in gradient.
            double limit_{1.};      ///< Step size limit.
            double damping_{1e-8};  ///< Damping factor.
            double tolerance_{magic::DEFAULT_IK_TOLERANCE};  ///< Tolerance for solving.
            std::size_t maxIter_{50};                        ///< Maximum iterations to use for solving.

            std::vector<TSRPtr> tsrs_;     ///< Set of TSRs
            std::vector<double> weights_;  ///< Weights on TSRs
            std::size_t dimension_{0};     ///< Total error dimension of set.

            Eigen::VectorXd upper_;  ///< Upper bounds on world configuration.
            Eigen::VectorXd lower_;  ///< Lower bounds on world configuration.
        };

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(TSRConstraint)
        /** \endcond */

        /** \class robowflex::darts::TSRConstraintPtr
            \brief A shared pointer wrapper for robowflex::darts::TSRConstraint. */

        /** \class robowflex::darts::TSRConstraintConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::TSRConstraint. */

        /** \brief An OMPL constraint for TSRs.
         *   Under the hood, creates a TSRSet that is used for all computation. Make sure that the robot state
         * space has all groups setup before creation of this constraint.
         */
        class TSRConstraint : public ompl::base::Constraint
        {
        public:
            /** \brief Constructor for a single TSR.
             *  \param[in] space Robot state space.
             *  \param[in] tsr TSR to use in constraint.
             */
            TSRConstraint(const StateSpacePtr &space, const TSRPtr &tsr);

            /** \brief Constructor for multiple TSRs.
             *  \param[in] space Robot state space.
             *  \param[in] tsrs TSRs to use in constraint.
             */
            TSRConstraint(const StateSpacePtr &space, const std::vector<TSRPtr> &tsrs);

            /** \brief Constructor for a TSRSet.
             *  \param[in] space Robot state space.
             *  \param[in] tsr TSRs to use in constraint.
             */
            TSRConstraint(const StateSpacePtr &space, const TSRSetPtr &tsr);

            /** \brief Get TSR set for this constraint.
             *  \return The constraint's TSR set.
             */
            TSRSetPtr getSet();

            void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::VectorXd> out) const override;

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::MatrixXd> out) const override;

            bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

            /** \brief Public options.
             */
            struct
            {
                bool use_gradient{false};
            } options;

        protected:
            StateSpacePtr space_;  ///< Robot state space.
            TSRSetPtr tsr_;        ///< Set of TSR constraints.
        };
    }  // namespace darts
}  // namespace robowflex

#endif
