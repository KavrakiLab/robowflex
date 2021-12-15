/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_BUILDER_
#define ROBOWFLEX_BUILDER_

#include <moveit/planning_pipeline/planning_pipeline.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>
#include <robowflex_library/id.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Planner);
    /** \endcond */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    /** \endcond */

    /** \class robowflex::MotionRequestBuilderPtr
        \brief A shared pointer wrapper for robowflex::MotionRequestBuilder. */

    /** \class robowflex::MotionRequestBuilderConstPtr
        \brief A const shared pointer wrapper for robowflex::MotionRequestBuilder. */

    /** \brief A helper class to build motion planning requests for a robowflex::Planner
     */
    class MotionRequestBuilder : public ID
    {
    public:
        /** \brief Constructor.
         *  \param[in] robot Robot to build planning problem for.
         */
        MotionRequestBuilder(const RobotConstPtr &robot);

        /** \brief Constructor. Set planner config and group as well.
         *  \param[in] robot Robot to build planning problem for.
         *  \param[in] group_name The motion planning group to build the request for.
         *  \param[in] planner_config Desired planning configuration to use.
         */
        MotionRequestBuilder(const RobotConstPtr &robot, const std::string &group_name,
                             const std::string &planner_config = "");

        /** \brief Constructor.
         *  \param[in] planner The motion planner to build a request for.
         *  \param[in] group_name The motion planning group to build the request for.
         *  \param[in] planner_config Desired planning configuration to use.
         */
        MotionRequestBuilder(const PlannerConstPtr &planner, const std::string &group_name,
                             const std::string &planner_config = "");

        /** \brief Copy Constructor.
         *  \param[in] other Request to copy.
         */
        MotionRequestBuilder(const MotionRequestBuilder &other);

        /** \brief Clone this request.
         *  \return A copy of this request.
         */
        MotionRequestBuilderPtr clone() const;

        /** \name Configuring Builder
            \{ */

        /** \brief Sets defaults.
         */
        void initialize();

        /** \brief Set the planning group to use for this request builder.
         *  \param[in] group_name Name of planning group.
         */
        void setPlanningGroup(const std::string &group_name);

        /** \brief Set the Robowflex planner to use.
         *  \param[in] planner Robowflex planner to build request for.
         */
        void setPlanner(const PlannerConstPtr &planner);

        /** \} */

        /** \name Starting Configurations
            \{ */

        /** \brief Set the start configuration from a vector \a joints.
         *  All joints are assumed to be specified and in the default order.
         *  \param[in] joints The values of the joints to set.
         */
        void setStartConfiguration(const std::vector<double> &joints);

        /** \brief Set the start configuration from a robot state.
         *  \param[in] state The robot state to set. Usually from robowflex::Robot::getScratchState().
         */
        void setStartConfiguration(const robot_state::RobotState &state);

        /** \brief Set the start configuration from a robot state.
         *  \param[in] state The robot state to set. Usually from robowflex::Robot::getScratchState().
         */
        void setStartConfiguration(const robot_state::RobotStatePtr &state);

        /** \brief Use the current scene state for the starting configuration.
         *  \param[in] scene Scene to use state from.
         */
        void useSceneStateAsStart(const SceneConstPtr &scene);

        /** \brief Attach an object to the current request start state. Uses \a object from \a scene, and
         *  modifies the underlying scene state. This uses `attachObject()` on the scene, so note that the
         *  attached object will have to be re-added to the scene.
         *  \param[in] scene Scene to use objects from. Will be modified.
         *  \param[in] object Object to attach.
         *  \return True on success, false on failure.
         */
        bool attachObjectToStart(ScenePtr scene, const std::string &object);

        /** \brief Attach an object to the current request start state. Uses \a object from \a scene, but does
         *  not modify the underlying scene. Be aware that attached objects can collide with themselves if not
         *  removed from the provided scene.
         *  \param[in] scene Scene to use objects from.
         *  \param[in] object Object to attach.
         *  \return True on success, false on failure.
         */
        bool attachObjectToStartConst(const SceneConstPtr &scene, const std::string &object);

        /** \} */

        /** \name Goals
            \{ */

        /** \brief Add a goal configuration from a vector \a joints.
         *  All joints are assumed to be specified and in the default order.
         *  \param[in] joints The values of the joints to set.
         */
        void addGoalConfiguration(const std::vector<double> &joints);

        /** \brief Add a goal configuration from a robot state.
         *  \param[in] state The robot state to set as pointer. Usually from
         * robowflex::Robot::getScratchState().
         */
        void addGoalConfiguration(const robot_state::RobotStatePtr &state);

        /** \brief Add a goal configuration from a robot state.
         *  \param[in] state The robot state to set ass reference. Usually from
         * robowflex::Robot::getScratchState().
         */
        void addGoalConfiguration(const robot_state::RobotState &state);

        /** \brief Add an IK query as a goal pose.
         *  \param[in] query IK query to construct goal from.
         */
        void addGoalFromIKQuery(const Robot::IKQuery &query);

        /** \brief Add a goal pose for the end-effector \a ee_name.
         *  Generates a sphere with radius \a tolerance as well as  orientation tolerances of \a tolerance
         *  from \a pose.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose.
         *  \param[in] pose The pose of the end-effector in \a base_frame.
         *  \param[in] tolerance The tolerance to put on the pose.
         */
        void addGoalPose(const std::string &ee_name, const std::string &base_name, const RobotPose &pose,
                         double tolerance = 0.001);

        /** \brief Add a goal region for an end-effector \a ee_name.
         *  Sets the position constraint from \a geometry at a pose \a pose, and the orientation constraint
         *  from \a orientation and XYZ Euler angle tolerances \a tolerances.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose and \a orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        void addGoalRegion(const std::string &ee_name, const std::string &base_name, const RobotPose &pose,
                           const GeometryConstPtr &geometry, const Eigen::Quaterniond &orientation,
                           const Eigen::Vector3d &tolerances);

        /** \brief Tiles some \a geometry around a \a pose in \a base_name for the end-effector \a ee_name.
         * The \a geometry is placed at \a offset from \a pose, and \a n copies are placed evenly rotated
         * about \a axis. The desired \a orientation is also rotated about the axis and set for each copy.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose and \a orientation.
         *  \param[in] pose The pose of the frame to be rotated about.
         *  \param[in] geometry The geometry describing the position constraint.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         *  \param[in] offset Offset of the goal \a geometry from \a pose.
         *  \param[in] axis Axis to rotation the goal \a geometry and \a orientation about in \a pose.
         *  \param[in] n Number of rotations (evenly divided around the circle).
         */
        void addGoalRotaryTile(const std::string &ee_name, const std::string &base_name,
                               const RobotPose &pose, const GeometryConstPtr &geometry,
                               const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances,
                               const RobotPose &offset, const Eigen::Vector3d &axis, unsigned int n);

        /** \brief Adds a set of regions to grasp a cylinder from the side. This function assumes the X-axis
         * of the end-effector frame \a ee_name points "forward" for grasping.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose.
         *  \param[in] pose The pose of the frame to be rotated about.
         *  \param[in] cylinder The cylinder to grasp.
         *  \param[in] distance The distance from the cylinder to place the regions.
         *  \param[in] depth The depth of boxes to create.
         *  \param[in] n The number of regions to create.
         */
        void addCylinderSideGrasp(const std::string &ee_name, const std::string &base_name,
                                  const RobotPose &pose, const GeometryConstPtr &cylinder, double distance,
                                  double depth, unsigned int n);

        /** \brief Set the goal configuration from a vector \a joints.
         *  All joints are assumed to be specified and in the default order.
         *  \param[in] joints The values of the joints to set.
         */
        void setGoalConfiguration(const std::vector<double> &joints);

        /** \brief Set the goal configuration from a robot state.
         *  \param[in] state The robot state to set as pointer. Usually from
         * robowflex::Robot::getScratchState().
         */
        void setGoalConfiguration(const robot_state::RobotStatePtr &state);

        /** \brief Set the goal configuration from a robot state.
         *  \param[in] state The robot state to set as reference. Usually from
         * robowflex::Robot::getScratchState().
         */
        void setGoalConfiguration(const robot_state::RobotState &state);

        /** \brief Set the goal pose from an IK query.
         *  \param[in] query IK query to construct goal from.
         */
        void setGoalFromIKQuery(const Robot::IKQuery &query);

        /** \brief Set a goal pose for the end-effector \a ee_name.
         *  Generates a sphere with radius \a tolerance as well as  orientation tolerances of \a tolerance
         *  from \a pose.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose.
         *  \param[in] pose The pose of the end-effector in \a base_frame.
         *  \param[in] tolerance The tolerance to put on the pose.
         */
        void setGoalPose(const std::string &ee_name, const std::string &base_name, const RobotPose &pose,
                         double tolerance = 0.001);

        /** \brief Set a goal region for an end-effector \a ee_name.
         *  Sets the position constraint from \a geometry at a pose \a pose, and the orientation constraint
         *  from \a orientation and XYZ Euler angle tolerances \a tolerances.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose and \a orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        void setGoalRegion(const std::string &ee_name, const std::string &base_name, const RobotPose &pose,
                           const GeometryConstPtr &geometry, const Eigen::Quaterniond &orientation,
                           const Eigen::Vector3d &tolerances);

        /** \brief Callback function that returns true if configuration is valid for goal set, false
         * otherwise.
         */

        using ConfigurationValidityCallback = std::function<bool(const robot_state::RobotState &)>;

        /** \brief Override the goals of this motion request with precomputed goal configurations (from the
         * specified regions).
         *
         *  That is, rather than a set of sampleable goal regions, the request will have \a n_samples goal
         * configurations, all sampled from the prior goal regions.
         *
         *  \param[in] n_samples Number of samples to precompute.
         *  \param[in] scene Scene to collision check against.
         *  \param[in] callback If provided, will only keep samples that are valid according to callback.
         */
        void precomputeGoalConfigurations(std::size_t n_samples, const ScenePtr &scene,
                                          const ConfigurationValidityCallback &callback = {});

        /** \brief Clears all goals.
         */
        void clearGoals();

        /** \} */

        /** \name Path Constraints
            \{ */

        /** \brief Set a pose constraint on the path.
         *  Sets the position constraint from \a geometry at a pose \a pose, and the orientation constraint
         *  from \a orientation and XYZ Euler angle tolerances \a tolerances.
         * \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose and \a orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        void addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                   const RobotPose &pose, const GeometryConstPtr &geometry,
                                   const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);

        /** \brief Set a position constraint on the path.
         *  Sets the position constraint from \a geometry at a pose \a pose.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         */
        void addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                       const RobotPose &pose, const GeometryConstPtr &geometry);

        /** \brief Set an orientation constraint on the path.
         *  Sets the orientation constraint from \a orientation and XYZ Euler angle tolerances \a tolerances.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a orientation.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        void addPathOrientationConstraint(const std::string &ee_name, const std::string &base_name,
                                          const Eigen::Quaterniond &orientation,
                                          const Eigen::Vector3d &tolerances);

        /** \} */

        /** \name Miscellaneous Settings
            \{ */

        /** \brief Set the planning configuration to use for the motion planning request.
         *  Attempts to match \a requested_config with the planner configuration offered by \a planner_
         *  that is the shortest configuration that contains \a requested_config as a substring. For example,
         *  specifying `RRTConnect` will match `RRTConnectkConfigDefault`, and specifying `RRT` will match
         *  `RRTkConfigDefault` and not `RRTConnectkConfigDefault`.
         *  \param[in] requested_config The planner config to find and use.
         *  \return True if the \a requested_config is found, false otherwise.
         */
        bool setConfig(const std::string &requested_config);

        /** \brief Set the allowed planning time in the request.
         *  \param[in] allowed_planning_time The allowed planning time.
         */
        void setAllowedPlanningTime(double allowed_planning_time);

        /** \brief Set the number of planning attemps in the request.
         *  \param[in] num_planning_attempts The required time for planning attempts.
         */
        void setNumPlanningAttempts(unsigned int num_planning_attempts);

        /** \brief Sets workspace bounds of the planning request.
         *  \param[in] wp The workspace parameters to use.
         */
        void setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp);

        /** \brief Sets workspace bounds of the planning request.
         *  \param[in] min XYZ vector of minimum workspace bounds.
         *  \param[in] max XYZ vector of maximum workspace bounds.
         */
        void setWorkspaceBounds(const Eigen::Ref<const Eigen::VectorXd> &min,
                                const Eigen::Ref<const Eigen::VectorXd> &max);

        /** \brief Swap the start and goal configurations.
         * This is only possible when a single joint goal is specified, otherwise an error is raised.
         *  \return True upon success, False otherwise.
         */
        bool swapStartWithGoal();

        /** \} */

        /** \name Getters
            \{ */

        /** \brief Get a reference to the currently built motion planning request.
         *  \return The motion planning request.
         */
        planning_interface::MotionPlanRequest &getRequest();

        /** \brief Get a const reference to the currently built motion planning request.
         *  \return The motion planning request.
         */
        const planning_interface::MotionPlanRequest &getRequestConst() const;

        /** \brief Get the start state of the request as a robot state.
         *  \return The start state.
         */
        robot_state::RobotStatePtr getStartConfiguration() const;

        /** \brief Get the goal state of the request as a robot state.
         *  \return The goal state.
         */
        robot_state::RobotStatePtr getGoalConfiguration() const;

        /** \brief Get a reference to the current path constraints on the motion planning request.
         *  \return The motion planning request.
         */
        moveit_msgs::Constraints &getPathConstraints();

        /** \brief Get the robot for this request.
         *  \return The robot.
         */
        const RobotConstPtr &getRobot() const;

        /** \brief Get the planner for this request.
         *  \return The planner. Will be null if not set.
         */
        const PlannerConstPtr &getPlanner() const;

        /** \brief Get the planning group.
         *  \return The current planning group used.
         */
        const std::string &getPlanningGroup() const;

        /** \brief Get the planner config.
         *  \return The current planner config used.
         */
        const std::string &getPlannerConfig() const;

        /** \} */

        /** \name IO
            \{ */

        /** \brief Serialize the motion planning request to a YAML file \a file.
         *  \param[in] file The name of the file to serialize the request to.
         *  \return True on success, false on failure.
         */
        bool toYAMLFile(const std::string &file) const;

        /** \brief Load a planning request from a YAML file \a file.
         *  \param[in] file The name of the file to load the request from.
         *  \return True on success, false on failure.
         */
        bool fromYAMLFile(const std::string &file);

        /** \} */

    private:
        const RobotConstPtr robot_;  ///< The robot to build the request for.

        PlannerConstPtr planner_;                     ///< The planner to build the request for.
        std::string group_name_;                      ///< The group to plan for.
        robot_model::JointModelGroup *jmg_{nullptr};  ///< The joint model group of the robot (from \a
                                                      ///< group_name_)

        planning_interface::MotionPlanRequest request_;  ///< The build request.

        static const std::string DEFAULT_CONFIG;  ///< Default planner configuration to use
    };
}  // namespace robowflex

#endif
