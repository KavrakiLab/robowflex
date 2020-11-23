/* Author: Constantinos Chamzas, Zachary Kingston */

#ifndef ROBOWFLEX_TRAJECTORY_
#define ROBOWFLEX_TRAJECTORY_

#include <tuple>
#include <functional>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/io/visualization.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    /** \brief A metric over robot states.
     */
    typedef std::function<double(const robot_state::RobotState &, const robot_state::RobotState &)>
        PathMetric;

    class Trajectory
    {
    public:
        /** \brief Constructor.
         *  \param[in] robot Robot to construct planning scene for.
         *  \param[in] group Planning group of the trajectory.
         */
        Trajectory(const RobotConstPtr &robot, const std::string &group);

        /** \brief Constructor.
         *  \param[in] robot Robot to construct planning scene for.
         *  \param[in] group Planning group of the trajectory.
         */
        Trajectory(robot_trajectory::RobotTrajectory &trajectory);

        /** \brief Set the planning scene to be the same as a message.
         *  \param[in] reference_state A full state that contains the values for all the joints
         *  \param[in] message Message used to set planning scene
         */
        void useMessage(const robot_state::RobotState &reference_state,
                        const moveit_msgs::RobotTrajectory &msg);

        /** \brief Set the planning scene to be the same as a message.
         *  \param[in] reference_state A full state that contains the values for all the joints.
         *  \param[in] message Message used to set planning scene
         */
        void useMessage(const robot_state::RobotState &reference_state,
                        const trajectory_msgs::JointTrajectory &msg);

        /** \brief Dump a trajectory to a file.
         *  \param[in] filename Trajectory filename.
         *  \return True on success.
         */
        bool toYAMLFile(const std::string &filename) const;

        /** \brief Read a trajectory from a file to a file.
         *  \param[in] reference_state A full state that contains the values for all the joints.
         *  \param[in] filename Trajectory filename.
         *  \return True on success.
         */
        bool fromYAMLFile(const robot_state::RobotState &reference_state, const std::string &filename);

        /** \name Getters and Setters
                    \{ */

        /** \brief Get a const reference to the trajectory.
         *  \return The trajectory.
         */
        const robot_trajectory::RobotTrajectoryPtr &getTajectoryConst() const;

        /** \brief Get a reference to the trajectory.
         *  \return The trajectory .
         */
        robot_trajectory::RobotTrajectoryPtr &getTrajectory();

        /** \brief Get the message that describes the trajectory.
         *  \return The trajectory message.
         */
        moveit_msgs::RobotTrajectory getMessage() const;

        /** \brief Get the message that describes the trajectory.
         *  \return The trajectory message.
         */
        std::size_t size() const;

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
         * */
        void interpolate(unsigned int count);

        /** \} */

        /** \name Metrics
            \{ */

        /** \brief Get the length of a path.
         *  Optionally, a metric can be specified other than the default (the L2 norm).
         *  \param[in] path The path to compute the length of.
         *  \param[in] metric An optional metric to use to compute the length of the path.
         *  \return Length of the path according to the metric.
         */
        double getLength(const PathMetric &metric = {}) const;

        /** \brief Checks if a path is correct.
         *  \param[in] scene Scene to check path against.
         *  \return True if the path is collision free in the scene.
         */
        bool isCorrect(const SceneConstPtr &scene) const;

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

    private:
        robot_trajectory::RobotTrajectoryPtr trajectory_;
    };
}  // namespace robowflex

#endif
