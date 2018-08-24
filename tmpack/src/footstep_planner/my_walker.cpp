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

#ifndef ROBOWFLEX_MY_WALKER_CPP
#define ROBOWFLEX_MY_WALKER_CPP

#include <robowflex_library/robowflex.h>
#include <robowflex_library/io/visualization.h>
#include <random>
#include <vector>

#include <tmpack/tmpack_interface.cpp>
#include <tmpack/my_walker.h>
#include <tmpack/utils/util.h>
#include <tmpack/utils/geom_2D.h>
#include <tmpack/calc_footsteps.cpp>

#define SHOW_FAILING_MARKER 0

namespace robowflex
{
    // TODO: want to get feedback on how the motion planner did

    // Domain semantics are implemented by using callbacks in the getTaskPlan
    // and planLinearly functions
    void MyWalker::MyWalkerConstraintHelper::_getTaskPlan_Callback()
    {
        last_foot_left = true;
    }

    void MyWalker::MyWalkerConstraintHelper::_getTaskPlan_Callback(
        std::vector<std::vector<footstep_planning::point_2D>> possible_plans)
    {
        my_possible_plans = possible_plans;
        last_foot_left = true;
    }

    void MyWalker::MyWalkerConstraintHelper::_planLinearly_Callback(
        MotionRequestBuilderPtr request, const std::vector<double> &task_op, RobotPtr robot,
        const std::vector<double> &joint_positions)
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
        std::cout << "left: " << robot->getLinkTF("r2/left_leg/gripper/tip").translation() << std::endl;
        std::cout << "right: " << robot->getLinkTF("r2/right_leg/gripper/tip").translation() << std::endl;
        std::cout << "moving: " << moving_tip_name << std::endl;

        // I think this works? It sets the orientation correctly. The pose is for a sphere so
        // it shouldn't matter that we have a rotation.
        Eigen::Quaterniond tip_orientation = Eigen::Quaterniond(tip_tf.rotation());

        Eigen::Vector3d feet_tolerance = Eigen::Vector3d(0.01, 0.01, 0.01);
        Eigen::Vector3d waist_tolerance = Eigen::Vector3d(0.005, 0.005, 0.005);

        // Keep one foot fixed
        request->addPathPoseConstraint(stationary_tip_name, "world", tip_tf, Geometry::makeSphere(0.01),
                                       tip_orientation, feet_tolerance);

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

    // We call this when a plan fails and we want to get a new plan for the same goal
    void MyWalker::MyWalkerConstraintHelper::_planUsingFeedback_Callback(
        std::vector<footstep_planning::point_2D> attempted_plan, size_t failed_index)
    {
    }

    void MyWalker::MyWalkerSceneGraphHelper::_getTaskPlan_Callback()
    {
        // do nothing
    }

    void MyWalker::MyWalkerSceneGraphHelper::_planLinearly_Callback(MotionRequestBuilderPtr request,
                                                                    const std::vector<double> &task_op)
    {
        // do nothing
    }

    // returns vector of joint poses
    std::vector<std::vector<double>> MyWalker::getTaskPlan()
    {
        // Benchmarking code. We loop through random locations and try to plan to them.
        double rand_x = uni_rnd_smpl_(rand_eng_) * 0.75;
        double rand_y = uni_rnd_smpl_(rand_eng_) * 1.5;
        return getTaskPlan(rand_x, rand_y, 0.0);
    }

    // returns vector of joint poses
    // the goal is to not have to build the motion requests by hand every time
    // TMP has a common pattern of using the last goal as the new start
    // However, we need a way to pass in new constraints for each step
    // For example, in walking the constraints cause the alternating legs to stay still
    std::vector<std::vector<double>> MyWalker::getTaskPlan(double x, double y, double yaw)
    {
        std::vector<std::vector<double>> my_plan;

        // is this good style? The superclass has a reference to these
        my_constraint_helper._getTaskPlan_Callback();
        my_scene_graph_helper._getTaskPlan_Callback();

        std::vector<std::vector<footstep_planning::point_2D>> all_foot_placements =
            my_step_planner.calculateFootPlacementsForTorso(
                points, points[9], footstep_planning::point_2D(x, y), footstep_planning::normalizeAngle(yaw),
                footstep_planning::foot::left, 5);

        std::cout << "We found: " << all_foot_placements.size() << " potential paths" << std::endl;
        for (auto pth : all_foot_placements)
        {
            std::cout << "=============" << std::endl;
            for (auto pt : pth)
            {
                std::cout << pt << std::endl;
            }
        }

        std::cout << "Finished printing paths" << std::endl;

        std::vector<footstep_planning::point_2D> foot_placements = all_foot_placements[0];

        std::cout << "Torso pose: < " << x << ", " << y << " >" << std::endl;

        std::cout << "Foot placements: " << std::endl;
        for (footstep_planning::point_2D p : foot_placements)
        {
            std::cout << p << std::endl;
            my_plan.push_back({p.x, p.y});
        }
        return my_plan;
    }

    // When a motion plan fails, this gets the next task plan to attempt
    std::vector<std::vector<double>> MyWalker::getNextTaskPlan(size_t failure)
    {
        std::vector<std::vector<double>> my_plan;
        return my_plan;
    }

    // Uses motion plan failure to plan
    std::vector<planning_interface::MotionPlanResponse>
    MyWalker::planUsingFeedback(std::vector<std::vector<double>> goals)
    {
        std::vector<planning_interface::MotionPlanResponse> responses;

        std::vector<double> next_start_joint_positions =
            request->getRequest().start_state.joint_state.position;

        // we manually specify these because the virtual_link isn't included in the above:
        std::vector<double> tmp = {1.98552, 0.0242871, 9.14127e-05, 4.8366e-06, -2.4964e-06, 1, -6.53607e-07};

        tmp.insert(tmp.end(), next_start_joint_positions.begin(), next_start_joint_positions.end());
        next_start_joint_positions = tmp;

        for (std::vector<double> goal_conf : goals)
        {
            // domain semantics can all be done here?
            // robot->setState(next_start_joint_positions);
            constraint_helper._planLinearly_Callback(request, goal_conf, robot, next_start_joint_positions);
            scene_graph_helper._planLinearly_Callback(request, goal_conf);

            // request->setConfig("CBiRRT2");
            for (size_t step_attempts = 0; step_attempts < MAX_STEP_ATTEMPTS; step_attempts++)
            {
                request->setStartConfiguration(robot->getScratchState());
                planning_interface::MotionPlanResponse response = planner->plan(scene, request->getRequest());

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
                // request->setConfig("PRM");
            }
            // stop once a motion fails
            if (responses.back().error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                break;
            }
        }
        return responses;
    }

    std::vector<planning_interface::MotionPlanResponse> MyWalker::plan()
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

            if(SHOW_FAILING_MARKER)
                rviz_helper.addMarker(x, y, z);
        }
        return res;
    }

    std::vector<planning_interface::MotionPlanResponse> MyWalker::planToPose(double x, double y, double yaw)
    {
        std::vector<std::vector<double>> goals = getTaskPlan(x, y, yaw);
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

            if(SHOW_FAILING_MARKER)
                rviz_helper.addMarker(x, y, z);
        }
        return res;
    }

    void MyWalker::setStartAndGoal(int s, int g)
    {
        start_index = s;
        goal_index = g;
    }

}  // namespace robowflex

#endif
