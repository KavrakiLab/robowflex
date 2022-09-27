/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_UR5_
#define ROBOWFLEX_UR5_

#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(UR5Robot);
    /* \endcond */

    /** \class robowflex::UR5RobotPtr
        \brief A shared pointer wrapper for robowflex::UR5Robot. */

    /** \class robowflex::UR5RobotConstPtr
        \brief A const shared pointer wrapper for robowflex::UR5Robot. */

    /** \brief Convenience class that describes the default setup for UR5 (with robotiq gripper and load cell)
     *  Will first attempt to load configuration and description from the robowflex_resources package.
     *  See https://github.com/KavrakiLab/robowflex_resources for this package.
     *  If this package is not available, then ur_description / robotiq packages will be used.
     */
    class UR5Robot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        UR5Robot();

        /** \brief Initialize the robot with manipulator kinematics.
         *  \return True on success, false on failure.
         */
        bool initialize();

    private:
        static const std::string DEFAULT_URDF;        ///< Default URDF
        static const std::string DEFAULT_SRDF;        ///< Default SRDF
        static const std::string DEFAULT_LIMITS;      ///< Default Limits
        static const std::string DEFAULT_KINEMATICS;  ///< Default kinematics

        static const std::string RESOURCE_URDF;        ///< URDF from robowflex_resources
        static const std::string RESOURCE_SRDF;        ///< SRDF from robowflex_resources
        static const std::string RESOURCE_LIMITS;      ///< Limits from robowflex_resources
        static const std::string RESOURCE_KINEMATICS;  ///< kinematics from robowflex_resources
    };

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(UR5OMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::UR5OMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::UR5OMPLPipelinePlanner. */

        /** \class robowflex::OMPL::UR5OMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::UR5OMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for UR5.
         */
        class UR5OMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            UR5OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

            /** \brief Initialize the planning context. All parameter provided are defaults.
             *  \param[in] settings Settings to set on the parameter server.
             *  \param[in] adapters Planning adapters to load.
             *  \return True on success, false on failure.
             */
            bool initialize(const Settings &settings = Settings(),
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string DEFAULT_CONFIG;   ///< Default planning configuration.
            static const std::string RESOURCE_CONFIG;  ///< Planning configuration from robowflex_resources.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
