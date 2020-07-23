/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_PATH_
#define ROBOWFLEX_PATH_

#include <tuple>
#include <functional>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    /** \brief A collection of functions that compute properties about computed robot trajectories.
     */
    namespace path
    {
        /** \brief A metric over robot states.
         */
        typedef std::function<double(const robot_state::RobotState &, const robot_state::RobotState &)>
            PathMetric;

        /** \brief Get the length of a path.
         *  Optionally, a metric can be specified other than the default (the L2 norm).
         *  \param[in] path The path to compute the length of.
         *  \param[in] metric An optional metric to use to compute the length of the path.
         *  \return Length of the path according to the metric.
         */
        double getLength(const robot_trajectory::RobotTrajectory &path, const PathMetric &metric = {});

        /** \brief Checks if a path is correct.
         *  \param[in] path Path to check.
         *  \param[in] scene Scene to check path against.
         *  \return True if the path is collision free in the scene.
         */
        bool isCorrect(const robot_trajectory::RobotTrajectory &path, const SceneConstPtr &scene);

        /** \brief Get the average, minimum, and maximum clearance of a path.
         *  \param[in] path The path to compute the clearance of.
         *  \param[in] scene Scene to compute clearance to.
         *  \return In order, the average, minimum, and maximum clearance of a path to a scene.
         */
        std::tuple<double, double, double> getClearance(const robot_trajectory::RobotTrajectory &path,
                                                        const SceneConstPtr &scene);

        /** \brief Get the smoothness of a path relative to some metric.
         *  See internal function documentation for details.
         *  \param[in] path The path to compute the smoothness of.
         *  \param[in] metric An optional metric to use to compute the length of the path segments.
         *  \return Smoothness of the path.
         */
        double getSmoothness(const robot_trajectory::RobotTrajectory &path, const PathMetric &metric = {});

        /** \brief Returns the joint positions from the last state in a planned trajectory in \a response.
         *  \param[in] path The trajectory to get the state from.
         *  \return A map of joint name to joint position of the last state in \a response.
         */
        std::map<std::string, double> getFinalPositions(const robot_trajectory::RobotTrajectory &path);

        /** \brief Computes the time parameterization of a path according to a desired max velocity or
         * acceleration.
         *  \param[in,out] path Geometric path to add time parameterization to.
         *  \param[in] max_velocity Maximum path velocity.
         *  \param[in] max_acceleration Maximum path acceleration.
         *  \return True on success, false on failure.
         */
        bool computeTimeParameterization(robot_trajectory::RobotTrajectory &path, double max_velocity = 1.,
                                         double max_acceleration = 1.);


        /** \brief Dump a trajectory to a file.
         *  \param[in] filename Trajectory filename.
         *  \param[in] path Path to dump.
         *  \return True on success.
         */
        bool toYAMLFile(const std::string &filename, robot_trajectory::RobotTrajectory &path);

    }  // namespace path
};     // namespace robowflex

#endif
