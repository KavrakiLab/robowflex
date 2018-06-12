#include "robowflex.h"
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

    class MyWalker : public TMPackInterface
    {
        footstep_planning::FootstepPlanner my_step_planner;
        std::vector<footstep_planning::point_2D> points;

        // returns vector of joint poses
        std::vector<std::vector<double>> getTaskPlan()
        {
            std::vector<std::vector<double>> my_plan;
            std::vector<double> goal = {-0.39, -0.69, -2.12, 2.82, -0.39, 0};
            my_plan.push_back(goal);
            goal = {0.39, -0.69, -2.12, 2.82, -0.39, 0};
            my_plan.push_back(goal);

            std::vector<footstep_planning::point_2D> foot_placements =
                my_step_planner.calculate_foot_placements(points, points[9], points[17], footstep_planning::foot::left);

            return my_plan;
        }

    public:
        int start_index, goal_index;

        MyWalker(const Robot &robot, const std::string &group_name, OMPL::OMPLPipelinePlanner &planner, Scene &scene,
                 MotionRequestBuilder &request, std::vector<double> &start)
          : TMPackInterface(robot, group_name, planner, scene, request, start)
        {
            // parse file
            // build graph
            //
            std::vector<footstep_planning::line_segment> line_segments;
            std::vector<std::string> line_names;
            footstep_planning::loadScene("/home/awells/Development/nasa_footstep_planning/scenes/iss.txt",
                                         &line_segments, &line_names);
            // store a map of points and their names

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

            start_index = 0;
            goal_index = 1;
            std::string cmd = "/home/awells/Development/nasa_footstep_planning/run_walker.sh " +
                              std::to_string(start_index) + " " + std::to_string(goal_index) + "\n";
            std::cout << "Calling: " << cmd << std::endl;
            int r = system(cmd.c_str());
        }

        void setStartAndGoal(int s, int g)
        {
            start_index = s;
            goal_index = g;
        }
    };

}  // namespace robowflex
