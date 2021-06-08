/* Author: Constantinos Chamzas */

#ifndef ROBOWFLEX_OMPL_TRAJECTORY
#define ROBOWFLEX_OMPL_TRAJECTORY

#include <robowflex_library/class_forward.h>

// Moveit
#include <moveit/robot_trajectory/robot_trajectory.h>

// OMPL
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Trajectory);
    /** \endcond */

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(OMPLTrajectory);
        /** \endcond */

        /** \class robowflex::OMPL::OMPLTrajectoryPtr
            \brief A shared pointer wrapper for robowflex::OMPL::OMPLTrajectory. */

        /** \class robowflex::OMPL::OMPLTrajectoryConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::OMPLTrajectory. */

        /** \brief OMPLTrajectory provides OMPL path utilities to the Trajectory class.
         */
        class OMPLTrajectory : public Trajectory
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to construct trajectory for.
             *  \param[in] group Planning group of the trajectory.
             */
            OMPLTrajectory(const RobotConstPtr &robot, const std::string &group);

            /** \brief Constructor.
             *  \param[in] trajectory Trajectory to initialize with.
             */
            OMPLTrajectory(robot_trajectory::RobotTrajectory &trajectory);

            /** \brief Converts to an OMPL Path given an OMPL \a simple_setup.
             *  \param[in] ss a simple setup structure that has the correct stateSpaceInformation.
             *  \return The Geometric Path equivalent of the trajectory.
             */
            ompl::geometric::PathGeometric toOMPLPath(const ompl::geometric::SimpleSetupPtr &ss);

            /** \brief Sets the trajectory from an OMPL Path and a reference state.
             *  \param[in] reference_state A full state that contains the values for all the joints.
             *  \param[in] path The geometric OMPL path to convert.
             */
            void fromOMPLPath(const robot_state::RobotState &reference_state,
                              const ompl::geometric::PathGeometric &path);
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
