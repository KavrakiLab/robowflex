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
    }  // namespace path
};     // namespace robowflex

#endif
