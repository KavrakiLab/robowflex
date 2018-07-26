/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MOVEGROUP_SERVICES_
#define ROBOWFLEX_MOVEGROUP_SERVICES_

namespace robowflex
{
    namespace movegroup
    {
        /** Creates instances of the move_group interface for things to use. */
        // class MoveGroupPool
        // {
        //     /** get movegroup by group */
        // };

        /** Executes a motion plan by forwarding it to movegroup */
        // class MoveGroupExecutor
        // {
        //     /** execute */
        // };

        /** Gets robot from movegroup configuration. has ability to push state to move_group */
        // class MoveGroupRobot : public Robot
        // {
        //     /** push (state) */
        // };

        /** Gets the scene from the planning scene monitor. has ability to push state to move_group */
        // class MoveGroupScene : public Scene
        // {
        //     /** pull / push */
        // };

        /** Does planning by forwarding request to move_group. */
        // class MoveGroupPlanner : public Planner
        // {
        // public:
        //     MoveGroupPlanner();
        //     ~MoveGroupPlanner();

        //     planning_interface::MotionPlanResponse
        //     plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override;

        //     const std::vector<std::string> getPlannerConfigs() const override;
        // };

        /** Listens for all incoming motion planning requests to move_group and does a call back.
         */
        // class MoveGroupMonitor
        // {
        //     /** setCallback in consturctor??? */
        // };
    }  // namespace movegroup
}  // namespace robowflex

#endif
