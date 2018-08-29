/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_BUILDER_
#define ROBOWFLEX_BUILDER_

#include <moveit/planning_pipeline/planning_pipeline.h>

#include <robowflex_library/class_forward.h>

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

        /** \brief Set a goal region for an end-effector \a ee_name.
         *  Sets the position constraint from \a geometry at a pose \a pose, and the orientation constraint
         *  from \a orientation and XYZ Euler angle tolerances \a tolerances.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        void setGoalRegion(const std::string &ee_name, const std::string &base_name,
                           const Eigen::Affine3d &pose, const GeometryConstPtr &geometry,
                           const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);

        /***/
        void addGoalCylindricalTile(const std::string &ee_name, const std::string &base_name,
                                    const Eigen::Affine3d &pose, const GeometryConstPtr &region,
                                    double distance, unsigned int n);

        /** \} */

        /** \name Path Constraints
            \{ */

        /** \brief Set a pose constraint on the path.
         *  Sets the position constraint from \a geometry at a pose \a pose, and the orientation constraint
         *  from \a orientation and XYZ Euler angle tolerances \a tolerances.
         * \param[in] ee_name The name of the end-effector link.
         * \param[in] base_name The frame of pose and orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        void addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                   const Eigen::Affine3d &pose, const GeometryConstPtr &geometry,
                                   const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);

        /** \brief Set a position constraint on the path.
         *  Sets the position constraint from \a geometry at a pose \a pose.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         */
        void addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                       const Eigen::Affine3d &pose, const GeometryConstPtr &geometry);

        /** \brief Set an orientation constraint on the path.
         *  Sets the orientation constraint from \a orientation and XYZ Euler angle tolerances \a tolerances.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
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