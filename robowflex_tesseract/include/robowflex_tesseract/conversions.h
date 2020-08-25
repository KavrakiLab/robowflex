/* Author: Carlos Quintero, Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_CONVERSIONS_
#define ROBOWFLEX_TESSERACT_CONVERSIONS_

#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/class_forward.h>
#include <tesseract_ros/kdl/kdl_env.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Robot)
    /** \endcond */

    namespace hypercube
    {
        /** \brief Add scene collision objects to a (previously initialized) KDL environment.
         *  \param[in] scene Scene to load.
         *  \param[out] env KDL environment to add scene objects to.
         *  \return True if the KDL environment was correctly loaded from scene.
         */
        bool sceneToTesseractEnv(const robowflex::SceneConstPtr &scene,
                                 tesseract::tesseract_ros::KDLEnvPtr &env);

        /** \brief Transform a \a robot_state to a vector representing joint values for the manipulator (in
         * the order given by \a manip_joint_names).
         *  \param[in] robot_state Robot state to be transformed.
         *  \param[in] manip_joint_names Joint names for the manipulator state.
         *  \param[out] manip_joint_values Manipulator's joint values representing \a robot_state.
         */
        void robotStateToManipState(const robot_state::RobotStatePtr &robot_state,
                                    const std::vector<std::string> &manip_joint_names,
                                    std::vector<double> &manip_joint_values);

        /** \brief Transform a tesseract \a waypoint (manip state) to robot \a state. Joint values for
         * non-manip joints are taken from \a env.
         *  \param[in] manip_state Tesseract manipulator state to be transformed.
         *  \param[in] manip Name of manipulator.
         *  \param[in] env KDL environment with the robot (and manipulator) information already loaded.
         *  \param[out] robot_state Robot state representing \a manip_state.
         */
        void manipStateToRobotState(const Eigen::Ref<const Eigen::VectorXd> &manip_state,
                                    const std::string &manip, const tesseract::tesseract_ros::KDLEnvPtr &env,
                                    robot_state::RobotStatePtr &robot_state);
        
        /** \brief Transform a tesseract trajectory to a robot \a trajectory.
         *  \param[in] tesseract_traj Tesseract trajectory to transform.
         *  \param[in] robot Robot \a tesseract_traj belongs to.
         *  \param[in] manip Name of manipulator.
         *  \param[in] env KDL environment with the robot (and manipulator) information already loaded.
         *  \param[out] trajectory Robot trajectory corresponding to \a tesseract_traj.
         */
        void tesseractTrajToRobotTraj(const tesseract::TrajArray &tesseract_traj, const RobotPtr &robot, const std::string &manip, const tesseract::tesseract_ros::KDLEnvPtr &env, robot_trajectory::RobotTrajectoryPtr &trajectory);

    }  // namespace hypercube
}  // namespace robowflex

#endif
