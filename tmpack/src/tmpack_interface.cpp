#ifndef TMPACK_INTERFACE_CPP
#define TMPACK_INTERFACE_CPP

#include <robowflex_library/robowflex.h>
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
        virtual void _planLinearly_Callback(MotionRequestBuilder &request, const std::vector<double> &task_op, Robot& robot, const std::vector<double> &joint_positions) = 0;
    };

    // Class to help manipulate the scene graph when running the plan linearly
    // algorithm. This should be useful for things like re-parenting an object
    // once it is grasped.
    class TMPSceneGraphHelper
    {
    public:
        // TMPSceneGraphHelper() = 0;
        virtual void _getTaskPlan_Callback() = 0;
        virtual void _planLinearly_Callback(MotionRequestBuilder &request, const std::vector<double> &task_op) = 0;
    };

    class TMPackInterface
    {
        Robot &robot;
        const std::string &group_name;
        OMPL::OMPLPipelinePlanner &planner;
        Scene &scene;
        MotionRequestBuilder &request;

        // We use these callbacks to implement domain semantics
        TMPConstraintHelper &constraint_helper;
        TMPSceneGraphHelper &scene_graph_helper;

        virtual std::vector<std::vector<double>> getTaskPlan() = 0;

        std::vector<planning_interface::MotionPlanResponse> plan_linearly(std::vector<std::vector<double>> goals)
        {
            // Instead we are using the start from the yaml file
            // request.setStartConfiguration(start);
            std::vector<planning_interface::MotionPlanResponse> responses;

            std::vector<double> next_start_joint_positions = request.getRequest().start_state.joint_state.position;

            //we manually specify these because the virtual_link isn't included in the above:
            std::vector<double> tmp = {1.97695540603, 0.135286119285, 0.0538594464644, 0.00469409498409, -0.0735915747411, -0.996745300836, 0.0325737756642};

            tmp.insert(tmp.end(), next_start_joint_positions.begin(), next_start_joint_positions.end());
            next_start_joint_positions = tmp;

            for (std::vector<double> goal_conf : goals)
            {
                // domain semantics can all be done here?
                constraint_helper._planLinearly_Callback(request, goal_conf, robot, next_start_joint_positions);
                scene_graph_helper._planLinearly_Callback(request, goal_conf);

                planning_interface::MotionPlanResponse response = planner.plan(scene, request.getRequest());
                responses.push_back(response);

                next_start_joint_positions = getFinalJointPositions(response);
                request.setStartConfiguration(next_start_joint_positions);
                // std::cout<<"goal constraints: "<<request.getRequest().goal_constraints[0]<<std::endl;
                // std::cout<<"path constraints: "<<request.getPathConstraints()<<std::endl;
            }
            return responses;
        }

    public:
        TMPackInterface(Robot &robot, const std::string &group_name, OMPL::OMPLPipelinePlanner &planner,
                        Scene &scene, MotionRequestBuilder &request, TMPConstraintHelper &constraint_helper,
                        TMPSceneGraphHelper &scene_graph_helper)
          : robot(robot)
          , group_name(group_name)
          , planner(planner)
          , scene(scene)
          , request(request)
          , constraint_helper(constraint_helper)
          , scene_graph_helper(scene_graph_helper)
        {
        }

        std::vector<planning_interface::MotionPlanResponse> plan()
        {
            std::vector<std::vector<double>> goals = getTaskPlan();
            return plan_linearly(goals);
        }
    };

}  // namespace robowflex

#endif
