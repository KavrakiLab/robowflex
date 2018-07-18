/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_FETCH_
#define ROBOWFLEX_FETCH_

#include <robowflex_library/macros.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(FetchRobot);
    /* \endcond */

    /** \class robowflex::FetchRobotPtr
        \brief A shared pointer wrapper for robowflex::FetchRobot. */

    /** \class robowflex::FetchRobotConstPtr
        \brief A const shared pointer wrapper for robowflex::FetchRobot. */

    /** \brief Convenience class that describes the default setup for Fetch (with robotiq gripper and load cell)
     */
    class FetchRobot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        FetchRobot();

        /** \brief Initialize the robot with manipulator kinematics.
         *  \return True on success, false on failure.
         */
        bool initialize();

    private:
        static const std::string URDF;        ///< Default URDF
        static const std::string SRDF;        ///< Default SRDF
        static const std::string LIMITS;      ///< Default Limits
        static const std::string KINEMATICS;  ///< Default kinematics
    };

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(FetchOMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::FetchOMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::FetchOMPLPipelinePlanner. */

        /** \class robowflex::OMPL::FetchOMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::FetchOMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for Fetch.
         */
        class FetchOMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            FetchOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

            /** \brief Initialize the planning context. All parameter provided are defaults.
             *  \param[in] config_file A YAML file containing OMPL planner configurations.
             *  \param[in] settings Settings to set on the parameter server.
             *  \param[in] plugin Planning plugin to load.
             *  \param[in] adapters Planning adapters to load.
             *  \return True on success, false on failure.
             */
            bool initialize(const Settings &settings = Settings(), const std::string &config_file = CONFIG,
                            const std::string &plugin = DEFAULT_PLUGIN,
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string CONFIG;  ///< Default planning configuration.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif

