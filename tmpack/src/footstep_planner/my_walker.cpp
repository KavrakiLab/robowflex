#include <robowflex_library/robowflex.h>
#include <random>
#include <vector>

#include "../tmpack_interface.cpp"
#include "utils/util.h"
#include "utils/geom_2D.h"
#include "calc_footsteps.cpp"

// #define GOAL_POSE                                                                                                      \
//     {                                                                                                                  \
//         1.07485, -0.019672, 0.000100924, 1.27794e-05, -3.73287e-06, 0.985502, 0.169662, -0.278892, -0.566762,          \
//             -0.11038, 1.4638, -0.512414, 1.8041, 1.45799, 0, 0, 0, -1.47759, -0.40627, 0.166451, 1.38375, 0.293776,    \
//             0.0480252, 1.57101, 8.88178e-16, 0, 0, -0.340968, 0.872665, -1.39626, -1.8326, -2.44346, 1.39626, 0, 0, 0, \
//             0, 0, 0, 0, 1.77636e-15, 0, 1.77636e-15, -8.88178e-16, 0, -8.88178e-16, 8.88178e-16, 0, 8.88178e-16,       \
//             8.88178e-16, 8.88178e-16, 0, 8.88178e-16, -0.872665, -1.39626, 1.8326, -2.44346, -1.39626, 8.88178e-16, 0, \
//             0, 0, 0, 0, 0, 0, 0, 8.88178e-16, 0, 0, 0, 0, 0, 0, -8.88178e-16, -2.66454e-15, -2.66454e-15, 1.77636e-15, \
//             -0.0872665, 1.77636e-15, 0                                                                                 \
//     }

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
            std::vector<std::vector<double>> my_foot_placements;
            size_t op_index = 0;
        public:
            MyWalkerConstraintHelper(){};
            void _getTaskPlan_Callback()
            {
                // do nothing
            }

            void _planLinearly_Callback(MotionRequestBuilder &request, const std::vector<double> &task_op,
                                        Robot &robot, const std::vector<double> &joint_positions)
            {
                // Hopefully z is correct height
                double x = 0, y = 0, z = -0.95;
                x = task_op[0];
                y = task_op[1];

                //the measurements for the walker are in a different frame:
                double tmp = x;
                x = 2 + y/84;
                y = -tmp/84;

                //x = 41.9/100;
                //y = -41.9/100;

                // Path constraint from r2_plan.yml. We'll want to alternate feet for these. For now,
                //we hope there's only the one important constraint.
                request.getPathConstraints().position_constraints.clear();
                request.getPathConstraints().orientation_constraints.clear();

                std::string moving_tip_name = "r2/left_leg/gripper/tip";
                std::string stationary_tip_name = "r2/right_leg/gripper/tip";
                if (!last_foot_left)
                {
                    moving_tip_name = "r2/right_leg/gripper/tip";
                    stationary_tip_name = "r2/left_leg/gripper/tip";
                }

                //Find the location of the stationary tip in the workspace
                robot.setState(joint_positions);
                Eigen::Affine3d tip_tf = robot.getLinkTF(stationary_tip_name);
                std::cout << "stat: "<< tip_tf.translation() << std::endl;
                std::cout << "mover: "<< robot.getLinkTF(moving_tip_name).translation() << std::endl;

                //I think this works? It sets the orientation correctly. The pose is for a sphere so
                //it shouldn't matter that we have a rotation.
                Eigen::Quaterniond tip_orientation = Eigen::Quaterniond(tip_tf.rotation());


                request.addPathPoseConstraint(stationary_tip_name, "world", tip_tf,
                                              Geometry(Geometry::ShapeType::SPHERE,
                                                       Eigen::Vector3d(0.1, 0.1, 0.1),
                                                       "my_sphere_for_constraint_2"),
                                              tip_orientation, Eigen::Vector3d(0.01, 0.01, 0.01));

                request.setGoalRegion(
                    moving_tip_name, "world",
                    Eigen::Affine3d(Eigen::Translation3d(x, y, z) * Eigen::Quaterniond::Identity()),
                    Geometry(Geometry::ShapeType::SPHERE, Eigen::Vector3d(0.1, 0.1, 0.1),
                             "my_sphere_for_constraint_1"),
                    Eigen::Quaterniond(0, 0, 1, 0), Eigen::Vector3d(0.01, 0.01, 0.01));

                last_foot_left = !last_foot_left;
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

            void _planLinearly_Callback(MotionRequestBuilder &request, const std::vector<double> &task_op)
            {
                // do nothing
            }
        } my_scene_graph_helper;

        footstep_planning::FootstepPlanner my_step_planner;
        std::vector<footstep_planning::point_2D> points;

        // returns vector of joint poses
        // the goal is to not have to build the motion requests by hand every time
        // TMP has a common pattern of using the last goal as the new start
        // However, we need a way to pass in new constraints for each step
        // For example, in walking the constraints cause the alternating legs to stay still
        std::vector<std::vector<double>> getTaskPlan()
        {
            std::vector<std::vector<double>> my_plan;
            // std::vector<double> goal = GOAL_POSE;
            // my_plan.push_back(goal);

            // is this good style? The superclass has a reference to these
            my_constraint_helper._getTaskPlan_Callback();
            my_scene_graph_helper._getTaskPlan_Callback();

            // Not currently used, just seeing if the interface works
            std::vector<footstep_planning::point_2D> foot_placements =
                my_step_planner.calculate_foot_placements(points, points[9], points[17],
                                                          footstep_planning::foot::left);

            // Benchmarking code. We'll loop through random locations and try to plan to them.
            // TODO: Get random pose for torso, plan to it and return plan
            double rand_x, rand_y;
            std::uniform_real_distribution<double> uni_rnd_smpl(-100, 100);
            std::default_random_engine re;
            rand_x = uni_rnd_smpl(re) * 0.75;
            rand_y = uni_rnd_smpl(re) * 1.5;

            //TODO: Actually use the plan
            my_plan.push_back({42, -72});

            return my_plan;
        }

    public:
        int start_index, goal_index;

        std::vector<double> goal_pose;

        // Loads the scene description and creates the graph we will use for planning
        MyWalker(Robot &robot, const std::string &group_name, OMPL::OMPLPipelinePlanner &planner,
                 Scene &scene, MotionRequestBuilder &request)
          : TMPackInterface(robot, group_name, planner, scene, request, my_constraint_helper,
                            my_scene_graph_helper)
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

        void setStartAndGoal(int s, int g)
        {
            start_index = s;
            goal_index = g;
        }
    };

}  // namespace robowflex
