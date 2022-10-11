/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_MOVEIT_PLANNING_
#define ROBOWFLEX_DART_MOVEIT_PLANNING_

#include <moveit_msgs/MotionPlanRequest.h>

#include <robowflex_dart/planning.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/tsr.h>

namespace robowflex::darts::conversions
{
    /** \brief Get workspace bounds from a planning request.
     *  \param[out] builder Builder to modify.
     *  \param[in] msg Planning request message.
     */
    void getWorkspaceBoundsFromMessage(darts::PlanBuilder &builder,
                                       const moveit_msgs::MotionPlanRequest &msg);

    /** \brief Set group for planning from message.
     *  \param[out] builder Builder to modify.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Planning request message.
     */
    void getGroupFromMessage(darts::PlanBuilder &builder, const std::string &robot_name,
                             const moveit_msgs::MotionPlanRequest &msg);

    /** \brief Get the start state from the request message.
     *  \param[out] builder Builder to modify.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Planning request message.
     */
    void getStartFromMessage(darts::PlanBuilder &builder, const std::string &robot_name,
                             const moveit_msgs::MotionPlanRequest &msg);

    /** \brief Get the set of path constraints from the request message.
     *  \param[out] builder Builder to modify.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Planning request message.
     */
    void getPathConstraintsFromMessage(darts::PlanBuilder &builder, const std::string &robot_name,
                                       const moveit_msgs::MotionPlanRequest &msg);

    /** \brief Get the goal constraints from the planning request message.
     *  \param[out] builder Builder to modify.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Planning request message.
     */
    ompl::base::GoalPtr getGoalFromMessage(darts::PlanBuilder &builder, const std::string &robot_name,
                                           const moveit_msgs::MotionPlanRequest &msg);

    /** \brief Get a TSR from an position constraint.
     *  \param[out] world World to use.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Position constraint.
     */
    TSRPtr fromPositionConstraint(darts::WorldPtr &world, const std::string &robot_name,
                                  const moveit_msgs::PositionConstraint &msg);

    /** \brief Get a TSR from an orientation constraint.
     *  \param[out] world World to use.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Orientation constraint.
     */
    TSRPtr fromOrientationConstraint(darts::WorldPtr &world, const std::string &robot_name,
                                     const moveit_msgs::OrientationConstraint &msg);

    /** \brief Get a joint region goal from a message.
     *  \param[out] builder Builder to modify.
     *  \param[in] msgs Joint constraint messages to create region goal.
     *  \return The joint region goal.
     */
    JointRegionGoalPtr fromJointConstraints(darts::PlanBuilder &builder,
                                            const std::vector<moveit_msgs::JointConstraint> &msgs);

    /** \brief Use all of the planning request message to setup motion planning.
     *  \param[out] builder Builder to modify.
     *  \param[in] robot_name Robot name to use message on.
     *  \param[in] msg Planning request message.
     */
    ompl::base::GoalPtr fromMessage(darts::PlanBuilder &builder, const std::string &robot_name,
                                    const moveit_msgs::MotionPlanRequest &msg);
}  // namespace robowflex::darts::conversions

#endif
