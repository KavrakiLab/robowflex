#include "robowflex.h"
#include <vector>

#include "../tmpack_interface.cpp"

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
        std::vector<std::vector<double>> getTaskPlan()
        {
            std::vector<std::vector<double>> my_plan;
            std::vector<double> goal = {-0.39, -0.69, -2.12, 2.82, -0.39, 0};
            my_plan.push_back(goal);
            goal = {0.39, -0.69, -2.12, 2.82, -0.39, 0};
            my_plan.push_back(goal);
            return my_plan;
        }

    public:
        int start_index, goal_index;

        MyWalker(const Robot &robot, const std::string &group_name,
                 OMPL::OMPLPipelinePlanner &planner, Scene &scene, MotionRequestBuilder &request,
                 std::vector<double> &start)
          : TMPackInterface(robot, group_name, planner, scene, request, start)
        {
            // parse file
            // build graph
            //
            start_index = 0;
            goal_index = 1;
            std::string cmd = "/home/awells/Development/nasa_footstep_planning/run_walker.sh " +
                              std::to_string(start_index) + " " + std::to_string(goal_index) + "\n";
            std::cout<<"Calling: "<<cmd<<std::endl;
            int ret = system(cmd.c_str());
        }
    };

}  // namespace robowflex
