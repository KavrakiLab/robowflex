/* Author: Juan D. Hernandez */

#ifndef ROBOWFLEX_COB4_
#define ROBOWFLEX_COB4_

#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Cob4Robot);
    /* \endcond */

    /** \class robowflex::Cob4RobotPtr
        \brief A shared pointer wrapper for robowflex::Cob4Robot. */

    /** \class robowflex::Cob4RobotConstPtr
        \brief A const shared pointer wrapper for robowflex::Cob4Robot. */

    /** \brief Convenience class that describes the default setup for Care-O-Bot4.
     *  Will first attempt to load configuration and description from the robowflex_resources package.
     *  See https://github.com/KavrakiLab/robowflex_resources for this package.
     *  If this package is not available, then fetch_description / fetch_moveit_config packages will be used.
     */
    class Cob4Robot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        Cob4Robot();

        /** \brief Initialize the robot with arm_left and arm_right kinematics.
         *  \return True on success, false on failure.
         */
        bool initialize();

        /** \brief Sets the base pose of the Cob4 robot (a virtual planar joint)
         *  \param[in] x The x position.
         *  \param[in] y The y position.
         *  \param[in] theta The angle.
         */
        void setBasePose(double x, double y, double theta);

        /** \brief Points the Cob4's head to a point in the world frame.
         *  \param[in] point The point to look at.
         */
        void pointHead(const Eigen::Vector3d &point);

        /** \brief Opens the Cob4's grippers.
         */
        void openGrippers();

        /** \brief Opens the Cob4's left gripper.
         */
        void openLeftGripper();

        /** \brief Opens the Cob4's right gripper.
         */
        void openRightGripper();

        /** \brief Closes the Cob4's grippers.
         */
        void closeGrippers();

        /** \brief Closes the Cob4's left gripper.
         */
        void closeLeftGripper();

        /** \brief Closes the Cob4's right gripper.
         */
        void closeRightGripper();

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
        ROBOWFLEX_CLASS_FORWARD(Cob4OMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::Cob4OMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::Cob4OMPLPipelinePlanner. */

        /** \class robowflex::OMPL::Cob4OMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::Cob4OMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for Cob4.
         */
        class Cob4OMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            Cob4OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

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
