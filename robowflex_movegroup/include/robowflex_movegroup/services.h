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

namespace robowflex
{
    /** \brief Move group interaction related classes and features.
     */
    namespace movegroup
    {
        /** \brief A helper class that allows for pulling and pushing of scenes, robots, and trajectories to
         *  move group.
         */
        class MoveGroupHelper
        {
        public:
            /** \brief A container struct for all relevant information about a motion planning request to move
             *  group.
             */
            struct Action
            {
                std::string id;                           ///< Goal ID.
                ScenePtr scene;                           ///< Scene used for planning.
                moveit_msgs::MotionPlanRequest request;   ///< Motion planning request.
                bool success;                             ///< Planning success.
                double time;                              ///< Planning time.
                moveit_msgs::RobotTrajectory trajectory;  ///< Planned trajectory on success.

                /** \brief Load a recorded action from a YAML file.
                 *  \param[in] filename Filename to load from.
                 *  \return True on success, false on failure.
                 */
                bool fromYAMLFile(const std::string &filename);

                /** \brief Save a recorded action to a YAML file.
                 *  \param[in] filename Filename to save as.
                 *  \return True on success, false on failure.
                 */
                bool toYAMLFile(const std::string &filename);
            };

            typedef std::function<void(Action &)> ResultCallback;

            /** \brief Constructor. Sets up service clients.
             *  \param[in] move_group Name of the move group namespace.
             */
            MoveGroupHelper(const std::string &move_group = MOVE_GROUP);

            /** \brief Destructor. Cleans up services.
             */
            ~MoveGroupHelper();

            /** \brief Sets a callback function that is called whenever a motion plan request is serviced by
             *  move group.
             *  \param[in] callback Callback function to call upon move group requests.
             */
            void setResultCallback(const ResultCallback &callback);

            /** \brief Executes a planned trajectory through move group.
             *  \param[in] path Path to execute.
             *  \return True on success, false on failure.
             */
            bool executeTrajectory(const robot_trajectory::RobotTrajectory &path);

            /** \brief Pulls the current robot state from move group.
             *  \param[out] robot Robot whose state to set.
             *  \return True on success, false on failure.
             */
            bool pullState(RobotPtr robot);

            /** \brief Pulls the current planning scene from move group.
             *  \param[out] scene Scene to set to the current scene observed by move group.
             *  \return True on success, false on failure.
             */
            bool pullScene(ScenePtr scene);

            /** \brief Pushes the current planning scene to move group.
             *  \param[in] scene Scene to use to set move group's current scene.
             *  \return True on success, false on failure.
             */
            bool pushScene(const SceneConstPtr &scene);

            /** \brief Clears the octomap in the planning scene of the movegroup.
             *  \return True on success, false on failure.
             */
            bool clearOctomap();

        private:
            /** \brief Callback function for a move group goal.
             *  \param[in] msg Goal message.
             */
            void moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal &msg);

            /** \brief Callback function for a move group result.
             *  \param[in] msg Result message.
             */
            void moveGroupResultCallback(const moveit_msgs::MoveGroupActionResult &msg);

            ros::NodeHandle nh_;          ///< Node handle.
            ros::Subscriber goal_sub_;    ///< Move group goal subscriber.
            ros::Subscriber result_sub_;  ///< Move group result subscriber.
            ros::ServiceClient gpsc_;     ///< Get planning scene service.
            ros::ServiceClient apsc_;     ///< Apply planning scene service.
            ros::ServiceClient co_;       ///< Clear octomap service.

            actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> eac_;  ///< Execute trajectory
                                                                                       ///< client.

            ResultCallback callback_;  ///< Callback function for move group results.

            std::map<std::string, Action> requests_;  ///< Move group requests

            RobotPtr robot_;  ///< Robot on the parameter server used by move group.

            static const std::string MOVE_GROUP;     ///< Name of move_group namespace.
            static const std::string GET_SCENE;      ///< Name of get scene service.
            static const std::string APPLY_SCENE;    ///< Name of apply scene service.
            static const std::string CLEAR_OCTOMAP;  ///< Name of clear octomap service.
            static const std::string EXECUTE;        ///< Name of execute trajectory service.
        };
    }  // namespace movegroup
}  // namespace robowflex

#endif
