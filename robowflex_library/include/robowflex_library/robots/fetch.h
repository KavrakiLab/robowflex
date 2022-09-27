/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_FETCH_
#define ROBOWFLEX_FETCH_

#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(FetchRobot);
    /* \endcond */

    /** \class robowflex::FetchRobotPtr
        \brief A shared pointer wrapper for robowflex::FetchRobot. */

    /** \class robowflex::FetchRobotConstPtr
        \brief A const shared pointer wrapper for robowflex::FetchRobot. */

    /** \brief Convenience class that describes the default setup for Fetch.
     *  Will first attempt to load configuration and description from the robowflex_resources package.
     *  See https://github.com/KavrakiLab/robowflex_resources for this package.
     *  If this package is not available, then fetch_description / fetch_moveit_config packages will be used.
     */
    class FetchRobot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        FetchRobot();

        /** \brief Initialize the robot with arm and arm_with_torso kinematics.
         *  \param[in] addVirtual flag to add virtual joint.
         *  \param[in] use_low_limits flag to use lower joint limits.
         *  \return True on success, false on failure.
         */
        bool initialize(bool addVirtual = true, bool use_low_limits = false);

        /** \brief Inserts the caster links if they don't exist.
         *  \param[in] doc urdf description to be processed.
         *  \return True on success.
         */
        bool addCastersURDF(tinyxml2::XMLDocument &doc);

        /** \brief Sets the base pose of the Fetch robot (a virtual planar joint)
         *  \param[in] x The x position.
         *  \param[in] y The y position.
         *  \param[in] theta The angle.
         */
        void setBasePose(double x, double y, double theta);

        /** \brief Points the Fetch's head to a point in the world frame.
         *  \param[in] point The point to look at.
         */
        void pointHead(const Eigen::Vector3d &point);

        /** \brief Opens the Fetch's gripper.
         */
        void openGripper();

        /** \brief Closes the Fetch's gripper.
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
        static const std::string RESOURCE_LIMITS_LOW;  ///< Lower limits from robowflex_resources
        static const std::string RESOURCE_KINEMATICS;  ///< kinematics from robowflex_resources
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
