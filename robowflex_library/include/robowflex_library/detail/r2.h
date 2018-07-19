/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_R2_
#define ROBOWFLEX_R2_

#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(R2Robot);
    /* \endcond */

    /** \class robowflex::R2RobotPtr
        \brief A shared pointer wrapper for robowflex::R2Robot. */

    /** \class robowflex::R2RobotConstPtr
        \brief A const shared pointer wrapper for robowflex::R2Robot. */

    /** \brief Convenience class that describes the default setup for R2 (R2C6 full).
     */
    class R2Robot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        R2Robot();

        /** \brief Initialize the robot with a few kinematics groups.
         *  \param[in] kinematics List of kinematics solvers for planning groups to load.
         *  \return True on success, false on failure.
         */
        bool initialize(const std::vector<std::string> kinematics);

        /** \brief Loads telemetry data from SMT from an HDF5 file into a robot trajectory.
         *  \param[in] filename Filename to load.
         *  \return A robot trajectory corresponding to the data.
         */
        robot_trajectory::RobotTrajectoryPtr loadSMTData(const std::string &filename);

    private:
        static const std::string URDF;                           ///< Default URDF
        static const std::string SRDF;                           ///< Default SRDF
        static const std::string LIMITS;                         ///< Default Limits
        static const std::string KINEMATICS;                     ///< Default kinematics
        static const std::string CACHED;                         ///< IK Cache location
        static const std::vector<std::string> SAMPLERS;          ///< Constraint samplers
        static const std::map<std::string, std::string> CREEPY;  ///< Faster IK chains
    };

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(R2OMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::R2OMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::R2OMPLPipelinePlanner. */

        /** \class robowflex::OMPL::R2OMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::R2OMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for R2 (R2C6 full).
         */
        class R2OMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            R2OMPLPipelinePlanner(const R2RobotPtr &robot, const std::string &name = "");

            /** \brief Initialize the planning context. All parameter provided are defaults.
             *  \param[in] config_file A YAML file containing OMPL planner configurations.
             *  \param[in] settings Settings to set on the parameter server.
             *  \param[in] plugin Planning plugin to load.
             *  \param[in] adapters Planning adapters to load.
             *  \return True on success, false on failure.
             */
            bool initialize(const std::string &config_file = CONFIG, const Settings &settings = Settings(),
                            const std::string &plugin = PLUGIN,
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string CONFIG;  ///< Default planning configuration.
            static const std::string PLUGIN;  ///< Default planning plugin.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
