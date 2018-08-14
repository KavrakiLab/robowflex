/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#include <tmpack/my_walker.h>
#include "footstep_planner/my_graph_tester.cpp"
#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/r2.h>
#include <robowflex_library/io/visualization.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>

#define NUM_ITERATIONS 1

using namespace robowflex;

void shutdown(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robowflex", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);

    // R2RobotPtr r2 = std::make_shared<R2Robot>();
    auto r2 = std::make_shared<R2Robot>();

    r2->initialize({"legsandtorso"});
    // ScenePtr scene = std::make_shared<Scene>(r2);
    auto scene = std::make_shared<Scene>(r2);

    // OMPL::R2OMPLPipelinePlannerPtr planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);

    planner->initialize();
    auto request = std::make_shared<MotionRequestBuilder>(planner, "legsandtorso");
    request->fromYAMLFile("package://robowflex_library/yaml/r2_start.yml");

    size_t time_spent = 0;
    size_t count = 0;
    size_t success_count = 0;
    ros::Rate rate(0.5);
    // std::vector<double> start = START_POSE; //We ignore this for the YAML start

    // For r2_start.yml
    std::vector<double> tmp = {1.98552, 0.0242871, 9.14127e-05, 4.8366e-06, -2.4964e-06, 1, -6.53607e-07};
    std::vector<double> start_joint_positions = request->getRequest().start_state.joint_state.position;
    tmp.insert(tmp.end(), start_joint_positions.begin(), start_joint_positions.end());
    start_joint_positions = tmp;

    IO::RVIZHelper rviz = IO::RVIZHelper(r2, "robonaut2");

    MyWalker walker(r2, "legsandtorso", planner, scene, request, rviz);

    // int a;
    // std::cin >> a;
    for (; count < NUM_ITERATIONS; count++)
    {
        size_t begin = ros::Time::now().nsec;

        // reset start position before each iteration
        r2->setState(start_joint_positions);
        std::vector<planning_interface::MotionPlanResponse> res = walker.plan();
        // planning_interface::MotionPlanResponse res = planner.plan(scene, request.getRequest());
        if (!res.empty() && res.back().error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            success_count++;
        }
        // else
        // {
        //     request->toYAMLFile("/home/awells/failed_motions/" + std::to_string(count) + ".yml");
        // }

        // rviz.updateTrajectories(res);
        // rviz.updateMarkers();

        time_spent += (ros::Time::now().nsec - begin);
    }

    // while (true)
    //     ros::spinOnce();

    std::cout << "Time spent: " << time_spent << std::endl;
    std::cout << "Number of runs: " << count << std::endl;
    std::cout << "Number of successful runs: " << success_count << std::endl;

    return 0;
}
