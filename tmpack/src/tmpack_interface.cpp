#ifndef TMPACK_INTERFACE_CPP
#define TMPACK_INTERFACE_CPP

#include <include/robowflex.h>
#include <ros/ros.h>
#include <vector>

namespace robowflex
{
    // Class to help add constraints to the task plan when we are running the
    // plan linearly algorithm. Some algorithms (e.g., footstep planning)
    // will require these constraints to be modified in unusual ways (e.g.,
    // alternating which feet hit are locked in place).
    class TMPConstraintHelper
    {
    public:
        // TMPConstraintHelper() = 0;
        virtual void _getTaskPlan_Callback() = 0;
        virtual void _planLinearly_Callback(MotionRequestBuilder &request) = 0;
    };

    // Class to help manipulate the scene graph when running the plan linearly
    // algorithm. This should be useful for things like re-parenting an object
    // once it is grasped.
    class TMPSceneGraphHelper
    {
    public:
        // TMPSceneGraphHelper() = 0;
        virtual void _getTaskPlan_Callback() = 0;
        virtual void _planLinearly_Callback(MotionRequestBuilder &request) = 0;
    };

    class TMPackInterface
    {
        const Robot &robot;
        const std::string &group_name;
        OMPL::OMPLPipelinePlanner &planner;
        Scene &scene;
        MotionRequestBuilder &request;
        std::vector<double> &real_start_state;

        // We use these callbacks to implement domain semantics
        TMPConstraintHelper &constraint_helper;
        TMPSceneGraphHelper &scene_graph_helper;

        virtual std::vector<std::vector<double>> getTaskPlan() = 0;

        std::vector<planning_interface::MotionPlanResponse> plan_linearly(std::vector<double> start,
                                                                          std::vector<std::vector<double>> goals)
        {
            request.setStartConfiguration(start);
            std::vector<planning_interface::MotionPlanResponse> responses;

            for (std::vector<double> goal_conf : goals)
            {
                request.setGoalConfiguration(goal_conf);
                // domain semantics can all be done here?
                constraint_helper._planLinearly_Callback(request);
                scene_graph_helper._planLinearly_Callback(request);

                responses.push_back(planner.plan(scene, request.getRequest()));
                request.setStartConfiguration(goal_conf);
            }
            return responses;
        }

    public:
        TMPackInterface(const Robot &robot, const std::string &group_name, OMPL::OMPLPipelinePlanner &planner,
                        Scene &scene, MotionRequestBuilder &request, std::vector<double> &start,
                        TMPConstraintHelper &constraint_helper, TMPSceneGraphHelper &scene_graph_helper)
          : robot(robot)
          , group_name(group_name)
          , planner(planner)
          , scene(scene)
          , request(request)
          , real_start_state(start)
          , constraint_helper(constraint_helper)
          , scene_graph_helper(scene_graph_helper)
        {
        }

        std::vector<planning_interface::MotionPlanResponse> plan()
        {
            std::vector<std::vector<double>> goals = getTaskPlan();
            return plan_linearly(real_start_state, goals);
        }
    };

}  // namespace robowflex

#endif
