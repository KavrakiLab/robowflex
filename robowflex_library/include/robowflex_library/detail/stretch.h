/* Author: Carlos Quintero-Peña */

#ifndef ROBOWFLEX_STRETCH_
#define ROBOWFLEX_STRETCH_

#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(StretchRobot);
    /* \endcond */

    /** \class robowflex::StretchRobotPtr
        \brief A shared pointer wrapper for robowflex::StretchRobot. */

    /** \class robowflex::StretchRobotConstPtr
        \brief A const shared pointer wrapper for robowflex::StretchRobot. */

    /** \brief Convenience class that describes the default setup for Stretch.
     *  Will first attempt to load configuration and description from the robowflex_resources package.
     *  See https://github.com/KavrakiLab/robowflex_resources for this package.
     *  If this package is not available, then stretch_description / stretch_moveit_config packages will be
     * used.
     */
    class StretchRobot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        StretchRobot();

        /** \brief Initialize the robot.
         *  \param[in] addVirtual Whether a virtual joint for the base should be added to the srdf descrition
         * or not.
         *  \param[in] addBaseManip Whether a base + manipulator group should be added to the srdf
         * representation or not. If set, two groups will be added, one for the base and one for the mobile
         * base manipulator.
         *  \return True on success, false on failure.
         */
        bool initialize(bool addVirtual = true, bool addBaseManip = false);

        /** \brief Sets the base pose of the Stretch robot (a virtual planar joint)
         *  \param[in] x The x position.
         *  \param[in] y The y position.
         *  \param[in] theta The angle.
         */
        void setBasePose(double x, double y, double theta);

        /** \brief Points the Stretch's head to a point in the world frame.
         *  \param[in] point The point to look at.
         */
        void pointHead(const Eigen::Vector3d &point);

        /** \brief Opens the Stretch's gripper.
         */
        void openGripper();

        /** \brief Closes the Stretch's gripper.
         */
        void closeGripper();

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
        ROBOWFLEX_CLASS_FORWARD(StretchOMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::StretchOMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::StretchOMPLPipelinePlanner. */

        /** \class robowflex::OMPL::StretchOMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::StretchOMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for Stretch.
         */
        class StretchOMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            StretchOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

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
