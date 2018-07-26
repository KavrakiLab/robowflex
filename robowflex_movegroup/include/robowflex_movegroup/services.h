/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MOVEGROUP_SERVICES_
#define ROBOWFLEX_MOVEGROUP_SERVICES_

#include <ros/node_handle.h>

#include <moveit_msgs/MoveGroupActionGoal.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <actionlib/client/simple_action_client.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>

#include <robowflex_library/macros.h>

namespace robowflex
{
    namespace movegroup
    {
        /** \brief A helper class that allows for pulling and pushing of scenes, robots, and trajectories to
         *  move group.
         */
        class MoveGroupHelper
        {
        public:
            typedef std::function<void(
                const SceneConstPtr &scene, const moveit_msgs::MotionPlanRequest &request,
                moveit_msgs::MoveItErrorCodes code,
                const robot_trajectory::RobotTrajectoryConstPtr &trajectory, double time)>
                ResultCallback;

            /** \brief Constructor. Sets up service clients.
             */
            MoveGroupHelper(const std::string &move_group = MOVE_GROUP);

            /** \brief Destructor. Cleans up services.
             */
            ~MoveGroupHelper();

            /** \brief Sets a callback function that is called whenever a motion plan request is serviced by
             *  move group. Callback is called with the planning scene, planning request, resulting error
             * code, and the trajectory and planning time upon success. \param[in] callback Callback function
             * to call upon move group requests.
             */
            void setResultCallback(const ResultCallback &callback);

            /** \brief Executes a planned trajectory through move group.
             *  \param[in] group Planning group to plan for.
             *  \param[in] path Path to execute.
             */
            void executeTrajectory(const std::string &group,
                                   const robot_trajectory::RobotTrajectory &path) const;

            /** \brief Pulls the current robot state from move group.
             *  \param[out] robot Robot whose state to set.
             */
            void pullState(RobotPtr &robot) const;

            /** \brief Pulls the current planning scene from move group.
             *  \param[out] scene Scene to set to the current scene observed by move group.
             */
            void pullScene(ScenePtr &scene) const;

            /** \brief Pushes the current planning scene to move group.
             *  \param[in] scene Scene to use to set move group's current scene.
             */
            void pushScene(const SceneConstPtr &scene) const;

        private:
            /** \brief Callback function for a move group goal.
             *  \param[in] Goal message.
             */
            void moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal &msg);

            /** \brief Callback function for a move group result.
             *  \param[in] Result message.
             */
            void moveGroupResultCallback(const moveit_msgs::MoveGroupActionResult &msg);

            ros::NodeHandle nh_;          ///< Node handle.
            ros::Subscriber goal_sub_;    ///< Move group goal subscriber.
            ros::Subscriber result_sub_;  ///< Move group result subscriber.
            ros::ServiceClient gpsc_;     ///< Get planning scene service.
            ros::ServiceClient apsc_;     ///< Apply planning scene service.

            actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> eac_;  ///< Execute trajectory
                                                                                       ///< client.

            ResultCallback callback_;  ///< Callback function for move group results.

            static const std::string MOVE_GROUP;   ///< Name of move_group namespace.
            static const std::string GET_SCENE;    ///< Name of get scene service.
            static const std::string APPLY_SCENE;  ///< Name of apply scene service.
            static const std::string EXECUTE;      ///< Name of execute trajectory service.
        };
    }  // namespace movegroup
}  // namespace robowflex

#endif
