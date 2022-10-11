/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_MOVEIT_ROBOT_
#define ROBOWFLEX_DART_MOVEIT_ROBOT_

#include <moveit_msgs/RobotState.h>

#include <robowflex_moveit/core/robot.h>

#include <robowflex_dart/robot.h>

namespace robowflex::darts::conversions
{
    /** \brief Convert a robowflex::Robot into a Dart robot.
     *  \param[in] robot Robot to convert.
     */
    darts::RobotPtr fromMoveItRobot(const robowflex::RobotConstPtr &robot);

    /** \brief Set the current state of this robot from a MoveIt message.
     *  \param[in] msg Message to set state to.
     */
    void setStateFromMoveItMsg(darts::RobotPtr &robot, const moveit_msgs::RobotState &msg);

    /** \brief Set a MoveIt message from the current state of this robot.
     *  \param[out] msg Message to set.
     */
    void setMoveItMsgFromState(const darts::RobotConstPtr &robot, moveit_msgs::RobotState &msg);

    /** \brief Set the current state of this robot from a MoveIt robot state.
     *  \param[in] state MoveIt state to set state to.
     */
    void setStateFromMoveItState(darts::RobotPtr &robot, const robot_state::RobotState &state);

    /** \brief Set a MoveIt robot state from the current state of this robot.
     *  \param[out] state MoveIt state to set state to.
     */
    void setMoveItStateFromState(const darts::RobotConstPtr &robot, robot_state::RobotState &state);

    /** \brief Set the state using a MoveIt Joint Group.
     *  \param[in] jmg Name of joint group.
     *  \param[in] joints Value of Joint Group to set.
     */
    void setStateFromMoveItJMG(darts::RobotPtr &robot, robot_state::RobotState &scratch,
                               const std::string &jmg, const std::vector<double> &joints);

    /** \brief Set the state using a MoveIt Joint Group.
     *  \param[in] jmg Name of joint group.
     *  \param[in] vec Value of Joint Group to set.
     */
    void setStateFromMoveItJMG(darts::RobotPtr &robot, robot_state::RobotState &scratch,
                               const std::string &jmg, const Eigen::Ref<const Eigen::VectorXd> &vec);

    /** \brief Set a MoveIt Joint Group state from the current state of the robot.
     *  \param[in] jmg Name of joint group.
     *  \param[out] joints Joint values to set.
     */
    void setMoveItJMGFromState(const darts::RobotConstPtr &robot, robot_state::RobotState &scratch,
                               const std::string &jmg, std::vector<double> &joints);

    /** \brief Set a MoveIt Joint Group state from the current state of the robot.
     *  \param[in] jmg Name of joint group.
     *  \param[out] vec Joint values to set.
     */
    void setMoveItJMGFromState(const darts::RobotConstPtr &robot, robot_state::RobotState &scratch,
                               const std::string &jmg, Eigen::Ref<Eigen::VectorXd> vec);

}  // namespace robowflex::darts::conversions

#endif
