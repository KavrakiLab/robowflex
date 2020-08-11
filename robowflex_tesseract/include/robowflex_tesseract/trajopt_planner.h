/* Author: Carlos Quintero */

#ifndef ROBOWFLEX_TRAJOPT_PLANNER_
#define ROBOWFLEX_TRAJOPT_PLANNER_

#include <robowflex_library/class_forward.h>
#include <robowflex_library/builder.h>

#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/ros_tesseract_utils.h>
#include <tesseract_ros/kdl/kdl_env.h>

#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(RobotTrajectory);
    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    /** \endcond */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(TrajOptPlanner);
    /** \endcond */

    /** \class robowflex::TrajOptPlannerPtr
        \brief A shared pointer wrapper for robowflex::TrajOptPlanner. */

    /** \class robowflex::TrajOptPlannerConstPtr
        \brief A const shared pointer wrapper for robowflex::TrajOptPlanner. */

    /** \brief Robowflex Tesseract TrajOpt Planner.
     */
    class TrajOptPlanner
    {
    public:
        /** \brief Options structure with parameter values for TrajOpt planner.
         */
        struct Options
        {
            int num_waypoints{20};                     ///< Number of waypoints.
            double dt_lower_lim{2.0};                  ///< 1/max_dt.
            double dt_upper_lim{100.0};                ///< 1/min_dt.
            bool start_fixed{false};                   ///< Whether to use the current env state as a fixed initial state.
            bool use_time{false};                      ///< Whether any cost/cnt use time.
            double init_info_dt{0.5};                  ///< Value of dt in init_info.
            double joint_vel_coeffs{5.0};              ///< Coefficients for joint_vel costs.
            int collision_gap{1};                      ///< For continuous collision avoidance, compute swept-volume between timestep t and t+gap.
            double default_safety_margin{0.025};       ///< Default safety margin for collision avoidance.
            double default_safety_margin_coeffs{50.0}; ///< Coefficients for default safety margin in collision cost/constraints.
            double pose_cnt_pos_coeffs{10.0};          ///< Coefficients for pose constraints (position).
            double pose_cnt_rot_coeffs{10.0};          ///< Coefficients for pose constraints (rotation).
            double joint_pos_cnt_coeffs{5.0};          ///< Coefficients for joint position constraints.
            double config_max_iter{100.0};             ///< Maximum number of iterations of the SCP.
        } options;

        /** \brief Constructor.
         *  \param[in] robot Robot to plan for.
         *  \param[in] group_name Name of the (joint) group to plan for.
         *  \param[in] manip Name of the (chain) group to plan for.
         */
        TrajOptPlanner(const RobotConstPtr &robot, const std::string &group_name, const std::string &manip);

        /** \name Set and get TrajOpt parameters
            \{*/

        /** \brief Sets initial trajectory for TrajOpt and sets init_type to GIVEN_TRAJ
         *  \param[in] init_trajectory Trajectory to initialize TrajOpt.
         */
        void setInitialTrajectory(const trajopt::TrajArray &init_trajectory);

        /** \brief Sets type of initialization to use for the trajectory optimization.
         * Current options are:
         * STATIONARY
         * JOINT_INTERPOLATED
         * The user can also provide their own trajectory using setInitialTrajectory(). In such case, there is no need to call setInitType().
         *  \param[in] init_type Type of initial trajectory to be used.
         */
        void setInitType(const trajopt::InitInfo::Type &init_type);

        /** \brief Gets the trajectory that resulted in the last call to plan().
         *  \return Last trajectory computed using plan().
         */
        const robot_trajectory::RobotTrajectoryPtr &getTrajectory() const;

        /** \brief Gets the link names in the tesseract KDL environment.
         *  \return Name of links in the KDL environment.
         */
        const std::vector<std::string> &getEnvironmentLinks() const;

        /** \brief Gets the link names of the env manipulator.
         *  \return Name of links in the manipulator.
         */
        const std::vector<std::string> &getManipulatorLinks() const;

        /** \brief Gets the joint names of the env manipulator.
         *  \return Name of joints in the manipulator.
         */
        const std::vector<std::string> &getManipulatorJoints() const;

        /** \} */

        /** \name Planning functions
            \{ */

        /** \brief Plans a motion given a \a request (start and goal configuration) and a \a scene.
         *  \param[in] scene A planning scene for the same \a robot_ to compute the plan in.
         *  \param[in] request The motion planning request to solve.
         *  \return True if a plan was successfully computed.
         */
        bool plan(const SceneConstPtr &scene, const MotionRequestBuilderPtr &request);

        /** \brief Plans a motion using a \a scene from \a start_state to a \a goal_state.
         *  \param[in] scene Scene to plan for.
         *  \param[in] start_state Start state for the \a robot_.
         *  \param[in] goal_state Goal state for the \a robot_.
         *  \return True if a plan was successfully computed.
         */
        bool plan(const SceneConstPtr &scene, const robot_state::RobotStatePtr &start_state,
                  const robot_state::RobotStatePtr &goal_state);

        /** \brief Plans a motion given a \a start_state, a cartesian \a goal_pose for a \a link and a \a
         * scene. \param[in] scene A planning scene for the same \a robot_ to compute the plan in. \param[in]
         * start_state Start state for the \a robot_. \param[in] goal_pose Cartesian goal pose for \a link.
         *  \return True if a plan was successfully computed.
         */
        bool plan(const SceneConstPtr &scene, const std::unordered_map<std::string, double> &start_state,
                  const Eigen::Isometry3d &goal_pose, const std::string &link);

        /** \brief Plans a motion given a \a start_pose for \a start_link and a \a goal_pose for \a goal_link.
         *  \param[in] scene A planning scene for the same \a robot_ to compute the plan in.
         *  \param[in] start_pose Cartesian start pose for \a start_link.
         *  \param[in] start_link Robot's link with \a start_pose.
         *  \param[in] goal_pose Cartesian goal pose for \a goal_link.
         *  \return True if a plan was successfully computed.
         */
        bool plan(const SceneConstPtr &scene, const Eigen::Isometry3d &start_pose,
                  const std::string &start_link, const Eigen::Isometry3d &goal_pose,
                  const std::string &goal_link);

        /** \} */

        /** \name Debugging Options
            \{ */

        /** \brief Sets whether a report file (for the optimization) should be written or not.
         *  \param[in] file_write_cb Whether to write the file or not.
         *  \param[in] file_path File path
         */
        void setWriteFile(bool file_write_cb, const std::string &file_path = "");

        /** \} */

    private:
        /** \brief Creates a TrajOpt problem construction info object with default values.
         *  \param[out] pci Pointer to problem construction info initialized.
         */
        void problemConstructionInfo(std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Adds collision avoidance cost to the trajectory optimization for all waypoints.
         *  \param[out] pci Pointer to problem construction info with collision avoidance added.
         */
        void addCollisionAvoidance(std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Adds a (joint) configuration constraint in the first waypoint taken from \a request.
         *  \param[in] request Request motion planning problem.
         *  \param[out] pci Pointer to problem construction info with start state constraint added.
         */
        void addStartState(const MotionRequestBuilderPtr &request,
                           std::shared_ptr<trajopt::ProblemConstructionInfo> &pci);

        /** \brief Adds a (joint) configuration constraint \a start_state in the first waypoint.
         *  \param[in] start_state Desired robot's start state.
         *  \param[out] pci Pointer to problem construction info with start state constraint added.
         */
        void addStartState(const robot_state::RobotStatePtr &start_state,
                           std::shared_ptr<trajopt::ProblemConstructionInfo> &pci);

        /** \brief Adds a (joint) configuration constraint \a start_state in the first waypoint.
         *  \param[in] start_state Desired start manipulator's joint names/values.
         *  \param[out] pci Pointer to problem construction info with start state constraint added.
         */
        void addStartState(const std::unordered_map<std::string, double> &start_state,
                           std::shared_ptr<trajopt::ProblemConstructionInfo> &pci);

        /** \brief Adds a cartesian pose constraint \a start_pose for \a link in the first waypoint.
         *  \param[in] start_pose Desired start cartesian pose of \a link.
         *  \param[in] link Link that will be constrained to be in \a start_pose in the first waypoint.
         *  \param[out] pci Pointer to problem construction info with start pose constraint added.
         */
        void addStartPose(const robowflex::RobotPose &start_pose, const std::string &link,
                          std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Adds a (joint) configuration constraint in the last waypoint taken from \a request.
         *  \param[in] request Request motion planning problem.
         *  \param[out] pci Pointer to problem construction info with goal state constraint added.
         */
        void addGoalState(const MotionRequestBuilderPtr &request,
                          std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Adds a (joint) configuration constraint \a goal_state in the last waypoint.
         *  \param[in] goal_state Desired robot's goal state.
         *  \param[out] pci Pointer to problem construction info with goal state constraint added.
         */
        void addGoalState(const robot_state::RobotStatePtr &goal_state,
                          std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Adds a (joint) configuration constraint \a goal_state in the last waypoint.
         *  \param[in] goal_state Desired goal manipulator's joint values.
         *  \param[out] pci Pointer to problem construction info with goal state constraint added.
         */
        void addGoalState(const std::vector<double> goal_state,
                          std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Adds a cartesian pose constraint \a goal_pose for \a link in the last waypoint.
         *  \param[in] goal_pose Desired goal cartesian pose of \a link.
         *  \param[in] link Link that will be constrained to be in \a goal_pose in the last waypoint.
         *  \param[out] pci Pointer to problem construction info with goal pose constraint added.
         */
        void addGoalPose(const Eigen::Isometry3d &goal_pose, const std::string &link,
                         std::shared_ptr<trajopt::ProblemConstructionInfo> &pci) const;

        /** \brief Calls TrajOpt \a solve() and updates \a trajectory_.
         *  \param[out] pci Pointer to problem construction info initialized.
         */
        bool solve(const std::shared_ptr<trajopt::ProblemConstructionInfo> &pci);

        /** \brief Updates \a trajectory_ based on the planner response.
         *  \param[in] response Tesseract planner response to get trajectory from.
         */
        void updateTrajFromTesseractRes(const tesseract::tesseract_planning::PlannerResponse &response);

        RobotConstPtr robot_;                              ///< Robot to plan for.
        robot_trajectory::RobotTrajectoryPtr trajectory_;  ///< Last trajectory generated by the planner.
        tesseract::tesseract_ros::KDLEnvPtr env_;          ///< KDL environment.
        std::string group_;      ///< Name of group to plan for.
        std::string manip_;      ///< Name of manipulator chain to plan check for collisions.
        bool cont_cc_{true};     ///< Use continuous collision checking.
        std::shared_ptr<std::ofstream> stream_ptr_;  ///< Debug file stream.
        std::string file_path_;   ///< Path of debug file
        bool file_write_cb_{false};                  ///< Whether to write a debug file or not.
        trajopt::InitInfo::Type init_type_{trajopt::InitInfo::Type::STATIONARY};  ///< Type of initial
                                                                                  ///< trajectory.
        trajopt::TrajArray initial_trajectory_;  ///< Initial trajectory (if any).
    };
}  // namespace robowflex

#endif
