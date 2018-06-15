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
        // std::vector<double> &real_start_state;

        // We use these callbacks to implement domain semantics
        TMPConstraintHelper &constraint_helper;
        TMPSceneGraphHelper &scene_graph_helper;

        virtual std::vector<std::vector<double>> getTaskPlan() = 0;

        std::vector<planning_interface::MotionPlanResponse> plan_linearly(std::vector<std::vector<double>> goals)
        {
            //Instead we are using the start from the yaml file
            //request.setStartConfiguration(start);
            std::vector<planning_interface::MotionPlanResponse> responses;

            for (std::vector<double> goal_conf : goals)
            {
                // request.setGoalConfiguration(goal_conf);
                // domain semantics can all be done here?
                constraint_helper._planLinearly_Callback(request);
                scene_graph_helper._planLinearly_Callback(request);


                /*
                // Path constraint from r2_plan.yml
                // We'll want to alternate feet for these
                request.addPathPoseConstraint(
                    "r2/right_leg/gripper/tip", "world",
                    Eigen::Affine3d(Eigen::Translation3d(1.52599927295, 0.291900102863, -1.10399987558) *
                                    Eigen::Quaterniond::Identity()),
                    Geometry(Geometry::ShapeType::SPHERE, Eigen::Vector3d(0.1, 0.1, 0.1), "my_sphere_for_constraint_1"),
                    Eigen::Quaterniond(-0.00173564729294, 0.999998493763, 1.02802058722e-07, 9.19840220243e-09),
                    Eigen::Vector3d(0.01, 0.01, 0.01));

                // Goal constraint from r2_plan.yml
                request.setGoalRegion(
                    "r2/left_leg/gripper/tip", "world",
                    Eigen::Affine3d(Eigen::Translation3d(0.458251403895, -0.202812539965, -0.921309716372) *
                                    Eigen::Quaterniond::Identity()),
                    Geometry(Geometry::ShapeType::SPHERE, Eigen::Vector3d(0.1, 0.1, 0.1), "my_sphere_for_constraint_1"),
                    Eigen::Quaterniond(1.2006283947e-06, 0.999999999998, -2.88426574161e-07, 1.82386512384e-06),
                    Eigen::Vector3d(0.01, 0.01, 0.01));

                responses.push_back(planner.plan(scene, request.getRequest()));
                // request.setStartConfiguration(goal_conf);
                */

                responses.push_back(planner.plan(scene, request.getRequest()));
            }
            return responses;
        }

    public:
        TMPackInterface(const Robot &robot, const std::string &group_name, OMPL::OMPLPipelinePlanner &planner,
                        Scene &scene, MotionRequestBuilder &request,
                        TMPConstraintHelper &constraint_helper, TMPSceneGraphHelper &scene_graph_helper)
          : robot(robot)
          , group_name(group_name)
          , planner(planner)
          , scene(scene)
          , request(request)
          // , real_start_state(start)
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
