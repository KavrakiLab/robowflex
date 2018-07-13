#ifndef ROBOWFLEX_MY_WALKER_CPP
#define ROBOWFLEX_MY_WALKER_CPP

#include <robowflex_library/robowflex.h>
#include <robowflex_library/io/visualization.h>
#include <random>
#include <vector>

#include "../tmpack_interface.cpp"
#include "utils/util.h"
#include "utils/geom_2D.h"
#include "calc_footsteps.cpp"

namespace robowflex
{
    // Things this should implement:

    // parse the PDDL file
    // plan to solve the PDDL
    // map operator to motion plan
    // map operator to scene graph change

    // want to get feedback on how the motion planner did

    // parses PDDL and solves
    class MyWalker : public TMPackInterface
    {
        // Domain semantics are implemented by using callbacks in the getTaskPlan
        // and planLinearly functions
        class MyWalkerConstraintHelper : public TMPConstraintHelper
        {
            bool last_foot_left = true;
            // std::vector<std::vector<double>> my_foot_placements;
            // size_t op_index = 0;

            std::vector<std::vector<footstep_planning::point_2D>> my_possible_plans;

        public:
            MyWalkerConstraintHelper(){};
            void _getTaskPlan_Callback()
            {
                last_foot_left = true;
            }

            void _getTaskPlan_Callback(std::vector<std::vector<footstep_planning::point_2D>> possible_plans)
            {
                my_possible_plans = possible_plans;
                last_foot_left = true;
            }

            void _planLinearly_Callback(MotionRequestBuilderPtr request, const std::vector<double> &task_op,
                                        RobotPtr robot, const std::vector<double> &joint_positions)
            {
                // Hopefully z is correct height
                double x = 0, y = 0, z = -0.95;
                x = task_op[0];
                y = task_op[1];
                // the measurements for the walker are in a different frame:
                double tmp = x;
                x = 2 + y / 84;
                y = -tmp / 84;

                request->getPathConstraints().position_constraints.clear();
                request->getPathConstraints().orientation_constraints.clear();

                std::string moving_tip_name = "r2/left_leg/gripper/tip";
                std::string stationary_tip_name = "r2/right_leg/gripper/tip";
                if (!last_foot_left)
                {
                    moving_tip_name = "r2/right_leg/gripper/tip";
                    stationary_tip_name = "r2/left_leg/gripper/tip";
                }

                // Find the location of the stationary tip in the workspace
                // robot.setState(joint_positions);
                Eigen::Affine3d tip_tf = robot->getLinkTF(stationary_tip_name);
                std::cout << "left: " << robot->getLinkTF("r2/left_leg/gripper/tip").translation()
                          << std::endl;
                std::cout << "right: " << robot->getLinkTF("r2/right_leg/gripper/tip").translation()
                          << std::endl;
                std::cout << "moving: " << moving_tip_name << std::endl;

                // I think this works? It sets the orientation correctly. The pose is for a sphere so
                // it shouldn't matter that we have a rotation.
                Eigen::Quaterniond tip_orientation = Eigen::Quaterniond(tip_tf.rotation());

                Eigen::Vector3d feet_tolerance = Eigen::Vector3d(0.01, 0.01, 0.01);
                Eigen::Vector3d waist_tolerance = Eigen::Vector3d(0.005, 0.005, 0.005);

                // Keep one foot fixed
                request->addPathPoseConstraint(stationary_tip_name, "world", tip_tf,
                                               Geometry::makeSphere(0.01), tip_orientation, feet_tolerance);

                // TODO, this should be based on global z
                // Keep the torso upright
                std::string waist_name = "r2/waist_center";
                auto waist_tf = robot->getRelativeLinkTF(stationary_tip_name, waist_name);
                request->addPathOrientationConstraint(waist_name, stationary_tip_name,
                                                      Eigen::Quaterniond(waist_tf.rotation()), waist_tolerance);

                request->setGoalRegion(
                    moving_tip_name, "world",
                    Eigen::Affine3d(Eigen::Translation3d(x, y, z) * Eigen::Quaterniond::Identity()),
                    Geometry::makeSphere(0.01), Eigen::Quaterniond(0, 0, 1, 0), feet_tolerance);

                last_foot_left = !last_foot_left;
            }

            //We call this when a plan fails and we want to get a new plan for the same goal
            void _planUsingFeedback_Callback(std::vector<footstep_planning::point_2D> attempted_plan, size_t failed_index) {

            }

        } my_constraint_helper;

        class MyWalkerSceneGraphHelper : public TMPSceneGraphHelper
        {
        public:
            MyWalkerSceneGraphHelper(){};
            void _getTaskPlan_Callback()
            {
                // do nothing
            }

            void _planLinearly_Callback(MotionRequestBuilderPtr request, const std::vector<double> &task_op)
            {
                // do nothing
            }
        } my_scene_graph_helper;

        footstep_planning::FootstepPlanner my_step_planner;
        std::vector<footstep_planning::point_2D> points;

        std::uniform_real_distribution<double> uni_rnd_smpl_ =
            std::uniform_real_distribution<double>(-100, 100);
        std::default_random_engine rand_eng_;

        // returns vector of joint poses
        // the goal is to not have to build the motion requests by hand every time
        // TMP has a common pattern of using the last goal as the new start
        // However, we need a way to pass in new constraints for each step
        // For example, in walking the constraints cause the alternating legs to stay still
        std::vector<std::vector<double>> getTaskPlan()
        {
            std::vector<std::vector<double>> my_plan;

            // is this good style? The superclass has a reference to these
            my_constraint_helper._getTaskPlan_Callback();
            my_scene_graph_helper._getTaskPlan_Callback();

            // Benchmarking code. We loop through random locations and try to plan to them.
            double rand_x = uni_rnd_smpl_(rand_eng_) * 0.75;
            double rand_y = uni_rnd_smpl_(rand_eng_) * 1.5;

            std::vector<footstep_planning::point_2D> foot_placements = my_step_planner.calculateFootPlacementsForTorso(
                points, points[9], footstep_planning::point_2D(rand_x, rand_y),
                0.0, footstep_planning::foot::left);

            std::cout << "Torso pose: < " << rand_x << ", " << rand_y << " >" << std::endl;

            std::cout << "Foot placements: " << std::endl;
            for (footstep_planning::point_2D p : foot_placements)
            {
                std::cout << p << std::endl;
                my_plan.push_back({p.x, p.y});
            }
            return my_plan;
        }


        std::vector<planning_interface::MotionPlanResponse>
        planUsingFeedback(std::vector<std::vector<double>> goals)
        {
            std::vector<planning_interface::MotionPlanResponse> responses;

            std::vector<double> next_start_joint_positions =
                request->getRequest().start_state.joint_state.position;

            // we manually specify these because the virtual_link isn't included in the above:
            std::vector<double> tmp = {1.98552,     0.0242871, 9.14127e-05, 4.8366e-06,
                                       -2.4964e-06, 1,         -6.53607e-07};

            tmp.insert(tmp.end(), next_start_joint_positions.begin(), next_start_joint_positions.end());
            next_start_joint_positions = tmp;

            for (std::vector<double> goal_conf : goals)
            {
                // domain semantics can all be done here?
                // robot->setState(next_start_joint_positions);
                constraint_helper._planLinearly_Callback(request, goal_conf, robot,
                                                         next_start_joint_positions);
                scene_graph_helper._planLinearly_Callback(request, goal_conf);

                for (size_t step_attempts = 0; step_attempts < MAX_STEP_ATTEMPTS; step_attempts++)
                {
                    request->setStartConfiguration(robot->getScratchState());
                    planning_interface::MotionPlanResponse response =
                        planner->plan(scene, request->getRequest());

                    std::cout << "planner finished" << std::endl;

                    // only update if the motion was successful:
                    if (response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                    {
                        responses.push_back(response);
                        std::map<std::string, double> named_joint_positions =
                            getFinalJointPositions(response);
                        robot->setState(named_joint_positions);
                        next_start_joint_positions = robot->getState();
                        break;
                    }
                    else if (step_attempts >= MAX_STEP_ATTEMPTS - 1)
                    {  // We always want to have some response
                        responses.push_back(response);
                    }
                }
                // stop once a motion fails
                if (responses.back().error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    break;
                }
            }
            return responses;
        }


    public:
        int start_index, goal_index;

        std::vector<double> goal_pose;

        // Loads the scene description and creates the graph we will use for planning
        MyWalker(RobotPtr robot, const std::string &group_name, OMPL::OMPLPipelinePlannerPtr planner,
                 ScenePtr scene, MotionRequestBuilderPtr request, IO::RVIZHelper &rviz_helper)
          : TMPackInterface(robot, group_name, planner, scene, request, my_constraint_helper,
                            my_scene_graph_helper, rviz_helper)
        {
            std::vector<footstep_planning::line_segment> line_segments;
            std::vector<std::string> line_names;
            footstep_planning::loadScene("/home/awells/Development/nasa_footstep_planning/scenes/iss.txt",
                                         &line_segments, &line_names);
            // we only use the end points and the centers
            for (size_t i = 0; i < line_segments.size(); i++)
            {
                auto l = line_segments[i];
                std::string i_s = line_names[i];
                points.push_back(footstep_planning::point_2D(l.x1, l.y1));
                points.push_back(footstep_planning::point_2D(l.x2, l.y2));
                points.push_back(footstep_planning::point_2D((l.x1 + l.x2) / 2, (l.y1 + l.y2) / 2));
            }

            my_step_planner.buildGraph(points);
        }


        std::vector<planning_interface::MotionPlanResponse> plan()
        {
            std::vector<std::vector<double>> goals = getTaskPlan();
            std::vector<planning_interface::MotionPlanResponse> res = planUsingFeedback(goals);
            if (res.back().error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                std::vector<double> p = goals.back();
                double x = 0, y = 0, z = -0.95;
                x = p[0];
                y = p[1];
                // the measurements for the walker are in a different frame:
                double tmp = x;
                x = 2 + y / 84;
                y = -tmp / 84;

                // rviz_helper.addMarker(x, y, z);
            }
            return res;
        }


        void setStartAndGoal(int s, int g)
        {
            start_index = s;
            goal_index = g;
        }
    };

}  // namespace robowflex

#endif