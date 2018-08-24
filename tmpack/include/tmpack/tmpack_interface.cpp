/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Andrew Wells */

#ifndef ROBOWFLEX_TMPACK_INTERFACE_CPP
#define ROBOWFLEX_TMPACK_INTERFACE_CPP

#include <robowflex_library/robowflex.h>
#include <robowflex_library/io/visualization.h>
#include <ros/ros.h>
#include <vector>

#define MAX_STEP_ATTEMPTS 2

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
        virtual void _planLinearly_Callback(MotionRequestBuilderPtr request,
                                            const std::vector<double> &task_op, RobotPtr robot,
                                            const std::vector<double> &joint_positions) = 0;
    };

    // Class to help manipulate the scene graph when running the plan linearly
    // algorithm. This should be useful for things like re-parenting an object
    // once it is grasped.
    class TMPSceneGraphHelper
    {
    public:
        // TMPSceneGraphHelper() = 0;
        virtual void _getTaskPlan_Callback() = 0;
        virtual void _planLinearly_Callback(MotionRequestBuilderPtr request,
                                            const std::vector<double> &task_op) = 0;
    };

    class TMPackInterface
    {
    protected:
        RobotPtr robot;
        const std::string &group_name;
        OMPL::OMPLPipelinePlannerPtr planner;
        ScenePtr scene;
        MotionRequestBuilderPtr request;

        IO::RVIZHelper &rviz_helper;

        // We use these callbacks to implement domain semantics
        TMPConstraintHelper &constraint_helper;
        TMPSceneGraphHelper &scene_graph_helper;

        virtual std::vector<std::vector<double>> getTaskPlan() = 0;

        std::vector<planning_interface::MotionPlanResponse>
        plan_linearly(std::vector<std::vector<double>> goals)
        {
            // Instead we are using the start from the yaml file
            // request.setStartConfiguration(start);
            std::vector<planning_interface::MotionPlanResponse> responses;

            std::vector<double> next_start_joint_positions =
                request->getRequest().start_state.joint_state.position;

            // we manually specify these because the virtual_link isn't included in the above:
            // For r2_plan.yml
            // std::vector<double> tmp = {1.97695540603,    0.135286119285,  0.0538594464644,
            // 0.00469409498409,
            //                            -0.0735915747411, -0.996745300836, 0.0325737756642};

            // For r2_start.yml
            std::vector<double> tmp = {1.98552,     0.0242871, 9.14127e-05, 4.8366e-06,
                                       -2.4964e-06, 1,         -6.53607e-07};

            tmp.insert(tmp.end(), next_start_joint_positions.begin(), next_start_joint_positions.end());
            next_start_joint_positions = tmp;

            for (std::vector<double> goal_conf : goals)
            {
                std::cout << "setting start state with: " << next_start_joint_positions.size() << " joints"
                          << std::endl;
                // std::vector<std::string> names = robot->getJointNames();
                // for(int i= 0; i <names.size(); i++) {
                //   std::cout<<names[i]<<": "<<next_start_joint_positions[i]<<std::endl;
                // }

                // domain semantics can all be done here?
                // robot->setState(next_start_joint_positions);
                constraint_helper._planLinearly_Callback(request, goal_conf, robot,
                                                         next_start_joint_positions);
                scene_graph_helper._planLinearly_Callback(request, goal_conf);

                std::cout << "state set" << std::endl;

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
                            path::getFinalPositions(*response.trajectory_);
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
        TMPackInterface(RobotPtr robot, const std::string &group_name, OMPL::OMPLPipelinePlannerPtr planner,
                        ScenePtr scene, MotionRequestBuilderPtr request,
                        TMPConstraintHelper &constraint_helper, TMPSceneGraphHelper &scene_graph_helper,
                        IO::RVIZHelper &rviz_helper)
          : robot(robot)
          , group_name(group_name)
          , planner(planner)
          , scene(scene)
          , request(request)
          , rviz_helper(rviz_helper)
          , constraint_helper(constraint_helper)
          , scene_graph_helper(scene_graph_helper)
        {
        }

        virtual std::vector<planning_interface::MotionPlanResponse> plan()
        {
            std::cout<<"We are calling the wrong plan() method!"<<std::endl;

            std::vector<std::vector<double>> goals = getTaskPlan();
            std::vector<planning_interface::MotionPlanResponse> res = plan_linearly(goals);
            if (res.back().error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                std::vector<double> p = goals.back();
                double x = 0, y = 0;//, z = -0.95;
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
    };

}  // namespace robowflex

#endif
