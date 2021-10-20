/* Author: Constantinos Chamzas, Zachary Kingston */

#ifndef ROBOWFLEX_TRAJECTORY_
#define ROBOWFLEX_TRAJECTORY_

#include <tuple>
#include <functional>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    /** \brief A metric over robot states.
     */
    using PathMetric =
        std::function<double(const robot_state::RobotState &, const robot_state::RobotState &)>;

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Trajectory);
    /** \endcond */

    /** \class robowflex::TrajectoryPtr
        \brief A shared pointer wrapper for robowflex::Trajectory. */

    /** \class robowflex::TrajectoryConstPtr
        \brief A const shared pointer wrapper for robowflex::Trajectory. */

    /** \brief  The Trajectory class is a wrapper around _MoveIt!_'s robot_trajectory::RobotTrajectory,
     * with extra convenience functions such interpolation and collision checking.
     * There are also utilities to load and save trajectories from YAML files (toYAMLFile() and
     * fromYAMLFile()).
     */
    class Trajectory
    {
    public:
        /** \brief Constructor for an empty trajectory.
         *  \param[in] robot Robot to construct trajectory for.
         *  \param[in] group Planning group of the trajectory.
         */
        Trajectory(const RobotConstPtr &robot, const std::string &group);

        /** \brief Constructor from moveit trajectory.
         *  \param[in] trajectory Trajectory to initialize with.
         */
        Trajectory(const robot_trajectory::RobotTrajectory &trajectory);

        /** \brief Constructor from moveit trajectory.
         *  \param[in] trajectory Trajectory to initialize with.
         */
        Trajectory(const robot_trajectory::RobotTrajectoryPtr trajectory);

        /** \name IO
            \{ */

        /** \brief Set the trajectory to be the same as a message.
         *  \param[in] reference_state A full state that contains the values for all the joints
         *  \param[in] msg Message used to set the trajectory
         */
        void useMessage(const robot_state::RobotState &reference_state,
                        const moveit_msgs::RobotTrajectory &msg);

        /** \brief Set the trajectory to be the same as a message.
         *  \param[in] reference_state A full state that contains the values for all the joints.
         *  \param[in] msg Message used to set the trajectory
         */
        void useMessage(const robot_state::RobotState &reference_state,
                        const trajectory_msgs::JointTrajectory &msg);

        /** \brief Dump a trajectory to a file.
         *  \param[in] filename Trajectory filename.
         *  \return True on success.
         */
        bool toYAMLFile(const std::string &filename) const;

        /** \brief Load a trajectory from a YAML file.
         *  \param[in] reference_state A full state that contains the values for all the joints.
         *  \param[in] filename Trajectory filename.
         *  \return True on success.
         */
        bool fromYAMLFile(const robot_state::RobotState &reference_state, const std::string &filename);

        /** \} */

        /** \name Getters and Setters
            \{ */

        /** \brief Get a const reference to the trajectory.
         *  \return The trajectory.
         */
        const robot_trajectory::RobotTrajectoryPtr &getTrajectoryConst() const;

        /** \brief Get a reference to the trajectory.
         *  \return The trajectory.
         */
        robot_trajectory::RobotTrajectoryPtr &getTrajectory();

        /** \brief Get the message that describes the trajectory.
         *  \return The trajectory message.
         */
        moveit_msgs::RobotTrajectory getMessage() const;

        /** \brief Returns the number of waypoints of the trajectory.
         *  \return The numbers of waypoints of the trajectory.
         */
        std::size_t getNumWaypoints() const;

        /** \} */

        /** \name Adding and Modifying States
            \{ */

        /** \brief Add a waypoint at the end of the trajectory.
         *  \param[in] state State to add at end of trajectory.
         *  \param[in] dt Time to this waypoint from previous.
         */
        void addSuffixWaypoint(const robot_state::RobotState &state, double dt = 1.);

        /** \} */

        /** \name Processing Functions
            \{ */

        /** \brief Computes the time parameterization of a path according to a desired max velocity or
         * acceleration.
         *  \param[in] max_velocity Maximum path velocity.
         *  \param[in] max_acceleration Maximum path acceleration.
         *  \return True on success, false on failure.
         */
        bool computeTimeParameterization(double max_velocity = 1., double max_acceleration = 1.);

        /** \brief Computes the time parameterization of a path according to a desired max velocity or
         * acceleration.
         *  \param[in] trajectory to compute time parameterization.
         *  \param[in] max_velocity Maximum path velocity.
         *  \param[in] max_acceleration Maximum path acceleration.
         *  \return True on success, false on failure.
         */
        static bool computeTimeParameterization(robot_trajectory::RobotTrajectory &trajectory,
                                                double max_velocity = 1., double max_acceleration = 1.);

        /** \brief Insert a number of states in a path so that the path is made up of exactly count states.
         * States are inserted uniformly (more states on longer segments). Changes are performed only if a
         * path has less than count states.
         * \param[in] count number of states to insert.
         */
        void interpolate(unsigned int count);

        /** \brief Converts a trajectory into a vector of position vectors. The values are in the same order
         * as reported by getJointNames(), which is consistent within MoveIt.
         * \return The trajectory in vector form.
         */
        std::vector<std::vector<double>> vectorize() const;

        /** \brief  Get the names of the variables that make up this trajectory, in the same order as in
         * MoveIt JointModelGroup.
         * \return A vector of joint names in order.
         */
        std::vector<std::string> getJointNames() const;

        /** \brief Adds a specified part of a trajectory to the end of the current trajectory. The default,
         *  when \a start_index or \a end_index are ommitted, is to add the whole trajectory.
         *  \param[in] source The trajectory containing the part to append to the end of current trajectory.
         *  \param[in] dt Time step between last point in current traj and first point of append traj.
         *  \param[in] start_index Index of first traj point of the part to append from the source traj.
         *  \param[in] end_index Index of last traj point of the part to append from the source traj.
         */
        Trajectory &append(const Trajectory &source, double dt, size_t start_index = 0,
                           size_t end_index = std::numeric_limits<std::size_t>::max());

        /** \} */

        /** \name Metrics
            \{ */

        /** \brief Get the length of a path.
         *  Optionally, a metric can be specified other than the default (the L2 norm).
         *  \param[in] metric An optional metric to use to compute the length of the path.
         *  \return Length of the path according to the metric.
         */
        double getLength(const PathMetric &metric = {}) const;

        /** \brief Checks if a path is collsion free.
         *  \param[in] scene Scene to collision check the path with.
         *  \return True if the path is collision free in the scene.
         */
        bool isCollisionFree(const SceneConstPtr &scene) const;

        /** \brief Get the average, minimum, and maximum clearance of a path.
         *  \param[in] scene Scene to compute clearance to.
         *  \return In order, the average, minimum, and maximum clearance of a path to a scene.
         */
        std::tuple<double, double, double> getClearance(const SceneConstPtr &scene) const;

        /** \brief Get the smoothness of a path relative to some metric.
         *  See internal function documentation for details.
         *  \param[in] metric An optional metric to use to compute the length of the path segments.
         *  \return Smoothness of the path.
         */
        double getSmoothness(const PathMetric &metric = {}) const;

        /** \brief Returns the joint positions from the last state in a planned trajectory in \a response.
         *  \return A map of joint name to joint position of the last state in \a response.
         */
        std::map<std::string, double> getFinalPositions() const;

        /** \} */

    protected:
        robot_trajectory::RobotTrajectoryPtr trajectory_;
    };
}  // namespace robowflex

#endif
