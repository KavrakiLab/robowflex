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

            std::cout<<"Goals: "<<goals.size()<<std::endl;

            for (std::vector<double> goal_conf : goals)
            {
                // request.setGoalConfiguration(goal_conf);
                // domain semantics can all be done here?
                constraint_helper._planLinearly_Callback(request);
                scene_graph_helper._planLinearly_Callback(request);

                // Path constraint from r2_plan.yml
                // We'll want to alternate feet for these
                // For now, we hope there's only the one important constraint.
                request.getPathConstraints().position_constraints.clear();
                request.getPathConstraints().orientation_constraints.clear();

                request.addPathPoseConstraint(
                    "r2/right_leg/gripper/tip", "world",
                    Eigen::Affine3d(Eigen::Translation3d(1.526, 0.2919, -1.104) *
                                    Eigen::Quaterniond::Identity()),
                    Geometry(Geometry::ShapeType::SPHERE, Eigen::Vector3d(0.1, 0.1, 0.1), "my_sphere_for_constraint_2"),
                    Eigen::Quaterniond(9.19840220243e-09, -0.00173565, 0.999998, 1.02802058722e-07),
                    Eigen::Vector3d(0.01, 0.01, 0.01));

                request.setGoalRegion(
                    "r2/left_leg/gripper/tip", "world",
                    Eigen::Affine3d(Eigen::Translation3d(0.458251, -0.202813, -0.92131) *
                                    Eigen::Quaterniond::Identity()),
                    Geometry(Geometry::ShapeType::SPHERE, Eigen::Vector3d(0.1, 0.1, 0.1), "my_sphere_for_constraint_1"),
                    Eigen::Quaterniond(1.82386512384e-06, 1.2006283947e-06, 0.999999999998, -2.88426574161e-07),
                    Eigen::Vector3d(0.01, 0.01, 0.01));

                planning_interface::MotionPlanResponse response = planner.plan(scene, request.getRequest());
                auto traj = response.trajectory_;
                responses.push_back(response);

                // std::cout<<"num variables: "<<traj->getLastWayPoint().getVariableCount()<<std::endl;
                // const double *joints_pntr = traj->getLastWayPoint().getVariablePositions();
                // std::vector<double> joints;
                // for(size_t i = 0; i < traj->getLastWayPoint().getVariableCount(); i++) {
                //   joints.push_back(*(joints_pntr+i));
                // }
                // request.setStartConfiguration(std::vector<double>(joints));

                moveit_msgs::MotionPlanResponse msg;
                response.getMessage(msg);
                std::vector<double> joint_positions = msg.trajectory.joint_trajectory.points.back().positions;
                request.setStartConfiguration(joint_positions);

                std::cout<<"goal constraints: "<<request.getRequest().goal_constraints[0]<<std::endl;
                std::cout<<"path constraints: "<<request.getPathConstraints()<<std::endl;
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
