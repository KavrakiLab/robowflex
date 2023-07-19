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

        /** \brief Adds a mobile manipulator group composed of one group for the mobile base and another one
         * for the manipulator.
         *  \param[in] base_group Name to be given to the mobile base group.
         *  \param[in] manip_group Name of the existing manipulator group.
         *  \param[in] base_manip_group Name to be given to
         * the mobile base manipulator group.
         */
        void setSRDFPostProcessAddMobileManipulatorGroup(const std::string &base_group,
                                                         const std::string &manip_group,
                                                         const std::string &base_manip_group);

        /** \brief Adds a mobile manipulator solver plugin to the kinematics configuration description.
         *  \param[in] base_manip_group Name of the base manipulator group.
         *  \param[in] search_resolution Search resolution parameter for the kinematics solver.
         *  \param[in] timeout Timeout parameter for the kinematics solver.
         */
        void setKinematicsPostProcessAddBaseManipulatorPlugin(const std::string &base_group,
                                                              const std::string &base_manip_group,
                                                              double search_resolution = 0.005,
                                                              double timeout = 0.1);

        /** \brief Initialize the robot.
         *  \param[in] addVirtual Whether a virtual joint for the base should be added to the srdf descrition
         * or not.
         *  \param[in] addBaseManip Whether a base + manipulator group should be added to the srdf
         * representation or not. If set, two groups will be added, one for the base and one for the mobile
         * base manipulator.
         *  \return True on success, false on failure.
         */
        bool initialize(bool addVirtual = true, bool addBaseManip = false, const std::string &mob_base_manip="mobile_base_manipulator", const std::string &manip="stretch_arm");

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
