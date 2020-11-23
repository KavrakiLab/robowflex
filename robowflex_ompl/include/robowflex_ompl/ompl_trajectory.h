/* Author: Constantinos Chamzas */

#ifndef ROBOWFLEX_OMPL_TRAJECTORY
#define ROBOWFLEX_OMPL_TRAJECTORY

#include <robowflex_library/class_forward.h>
#include <robowflex_library/trajectory.h>

// OMPL
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(OMPLTrajectory);
        /** \endcond */

        /** \class robowflex::OMPL::OMPLInterfacePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::OMPLInterfacePlanner. */

        /** \class robowflex::OMPL::OMPLInterfacePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::OMPLInterfacePlanner. */

        /** \brief A planner that directly uses \a MoveIt!'s OMPL planning interface.
         */
        class OMPLTrajectory : public Trajectory
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to construct planning scene for.
             *  \param[in] group Planning group of the trajectory.
             */
            OMPLTrajectory(const RobotConstPtr &robot, const std::string &group);

            /** \brief Constructor.
             *  \param[in] robot Robot to construct planning scene for.
             *  \param[in] group Planning group of the trajectory.
             */
            OMPLTrajectory(robot_trajectory::RobotTrajectory &trajectory);

            /** \brief Converts to an OMPL Path given an OMPL \a simple_setup.
             *  \param[in] ss a simple setup structure that has the correct stateSpaceInformation.
             *  \return The Geometric Path equivalent of the trajectory.
             */
            ompl::geometric::PathGeometric toOMPLPath(const ompl::geometric::SimpleSetupPtr &ss);
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
