/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_BUILDER_
#define ROBOWFLEX_BUILDER_

#include <moveit/planning_pipeline/planning_pipeline.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
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
    class MotionRequestBuilder
    {
    public:
        /** \brief Constructor.
         *  \param[in] planner The motion planner to build a request for.
         *  \param[in] group_name The motion planning group to build the request for.
         */
        MotionRequestBuilder(const PlannerConstPtr &planner, const std::string &group_name);

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
        void setStartConfiguration(const robot_state::RobotStatePtr &state);

        /** \} */

        /** \name Goals
            \{ */

        /** \brief Set the goal configuration from a vector \a joints.
         *  All joints are assumed to be specified and in the default order.
         *  \param[in] joints The values of the joints to set.
         */
        void setGoalConfiguration(const std::vector<double> &joints);

        /** \brief Set the goal configuration from a robot state.
         *  \param[in] state The robot state to set. Usually from robowflex::Robot::getScratchState().
         */
        void setGoalConfiguration(const robot_state::RobotStatePtr &state);

        /** \brief Set a goal pose for the end-effector \a ee_name.
         *  Generates a sphere with radius \a tolerance as well as orientation tolerances of \a tolerance from \a pose.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The name of the frame of reference of \a pose.
         *  \param[in] pose The pose of the end-effector in \a base_frame.
         *  \param[in] tolerance The tolerance to put on the pose.
         */
        void setGoalPose(const std::string &ee_name, const std::string &base_name, const RobotPose &pose, double tolerance = 0.001);

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

        /** \brief Sets workspace bounds of the planning request.
         *  \param[in] wp The workspace parameters to use.
         */
        void setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp);

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

        /** \brief Get a reference to the current path constraints on the motion planning request.
         *  \return The motion planning request.
         */
        moveit_msgs::Constraints &getPathConstraints();

        /** \} */

        /** \name IO
            \{ */

        /** \brief Serialize the motion planning request to a YAML file \a file.
         *  \param[in] file The name of the file to serialize the request to.
         *  \return True on success, false on failure.
         */
        bool toYAMLFile(const std::string &file);

        /** \brief Load a planning request from a YAML file \a file.
         *  \param[in] file The name of the file to load the request from.
         *  \return True on success, false on failure.
         */
        bool fromYAMLFile(const std::string &file);

        /** \} */

    private:
        const PlannerConstPtr planner_;            ///< The planner to build the request for.
        const RobotConstPtr robot_;                ///< The robot to build the request for (from \a planner_)
        const std::string group_name_;             ///< The group to plan for.
        const robot_model::JointModelGroup *jmg_;  ///< The joint model group of the robot (from \a
                                                   ///< group_name_)

        planning_interface::MotionPlanRequest request_;  ///< The build request.

        static const std::vector<std::string> DEFAULT_CONFIGS;  ///< Default planner configurations to use
    };
}  // namespace robowflex

#endif
