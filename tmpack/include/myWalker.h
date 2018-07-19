#ifndef ROBOWFLEX_MY_WALKER_H
#define ROBOWFLEX_MY_WALKER_H

#include <robowflex_library/robowflex.h>
#include <robowflex_library/io/visualization.h>
#include <random>
#include <vector>

#include "../src/tmpack_interface.cpp"
#include "../src/footstep_planner/utils/util.h"
#include "../src/footstep_planner/utils/geom_2D.h"
#include "../src/footstep_planner/calc_footsteps.cpp"


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
        
        footstep_planning::FootstepPlanner my_step_planner;
        std::vector<footstep_planning::point_2D> points;

        std::list<std::list<footstep_planning::point_2D>> list_of_paths;

        std::uniform_real_distribution<double> uni_rnd_smpl_ =
            std::uniform_real_distribution<double>(-100, 100);
        std::default_random_engine rand_eng_;

        int start_index, goal_index;
        std::vector<double> goal_pose;

        // Domain semantics are implemented by using callbacks in the getTaskPlan
        // and planLinearly functions
        class MyWalkerConstraintHelper : public TMPConstraintHelper
        {
            bool last_foot_left = true;
            std::vector<std::vector<footstep_planning::point_2D>> my_possible_plans;

        public:
            MyWalkerConstraintHelper(){};
            void _getTaskPlan_Callback();

            void _getTaskPlan_Callback(std::vector<std::vector<footstep_planning::point_2D>> possible_plans);

            void _planLinearly_Callback(MotionRequestBuilderPtr request, const std::vector<double> &task_op,
                                        RobotPtr robot, const std::vector<double> &joint_positions);

            // We call this when a plan fails and we want to get a new plan for the same goal
            void _planUsingFeedback_Callback(std::vector<footstep_planning::point_2D> attempted_plan,
                                             size_t failed_index);
        } my_constraint_helper;
        
        class MyWalkerSceneGraphHelper : public TMPSceneGraphHelper
        {
        public:
            MyWalkerSceneGraphHelper(){};
            void _getTaskPlan_Callback();
            void _planLinearly_Callback(MotionRequestBuilderPtr request, const std::vector<double> &task_op);
        } my_scene_graph_helper;


        // returns vector of joint poses
        std::vector<std::vector<double>> getTaskPlan();

        // returns vector of joint poses
        // the goal is to not have to build the motion requests by hand every time
        // TMP has a common pattern of using the last goal as the new start
        // However, we need a way to pass in new constraints for each step
        // For example, in walking the constraints cause the alternating legs to stay still
        std::vector<std::vector<double>> getTaskPlan(double x, double y, double yaw);
        
        // When a motion plan fails, this gets the next task plan to attempt
        std::vector<std::vector<double>> getNextTaskPlan(size_t failure);

        // Uses motion plan failure to plan
        std::vector<planning_interface::MotionPlanResponse> planUsingFeedback(std::vector<std::vector<double>> goals);

    public:
        // Loads the scene description and creates the graph we will use for planning
        MyWalker(RobotPtr robot, const std::string &group_name, OMPL::OMPLPipelinePlannerPtr planner,
                 ScenePtr scene, MotionRequestBuilderPtr request, IO::RVIZHelper &rviz_helper)
          : TMPackInterface(robot, group_name, planner, scene, request, my_constraint_helper,
                            my_scene_graph_helper, rviz_helper)
        {
            std::vector<footstep_planning::line_segment> line_segments;
            std::vector<std::string> line_names;

            footstep_planning::loadScene(IO::resolvePath("package://tmpack/scenes/iss.txt"),
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

        // Do some (currently random) plan
        std::vector<planning_interface::MotionPlanResponse> plan() override;

        // Get a plan to the specified torso pose (assumes z is irrelevant)
        std::vector<planning_interface::MotionPlanResponse> planToPose(double x, double y, double yaw);

        void setStartAndGoal(int s, int g);
    };

}  // namespace robowflex

#endif