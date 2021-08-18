/* Author: Juan D. Hernandez */

#include <robowflex_library/log.h>
#include <robowflex_library/detail/cob4.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file cob4_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * COB4 robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, the pose goal, and motion plan
 * displayed in RViz.
 */

static const std::string LEFT_ARM = "arm_left";
static const std::string RIGHT_ARM = "arm_right";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Care-O-Bot4 robot.
    auto cob4 = std::make_shared<Cob4Robot>();
    cob4->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(cob4);

    // Load kinematics for the COB4 robot.
    cob4->loadKinematics(RIGHT_ARM);
    cob4->loadKinematics(LEFT_ARM);

    // Extend the arms.
    cob4->setGroupState(RIGHT_ARM, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    cob4->setGroupState(LEFT_ARM, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Create the default planner for the COB4.
    auto planner = std::make_shared<OMPL::Cob4OMPLPipelinePlanner>(cob4);
    planner->initialize();

    // Create a motion planning request with a joint position goal for the right arm.
    MotionRequestBuilder request_right_arm(planner, RIGHT_ARM);
    request_right_arm.setStartConfiguration(cob4->getScratchState());

    // Create a motion planning request with a pose goal to fold the right arm.
    cob4->setGroupState(RIGHT_ARM, {2.69, 1.70, -0.91, 1.50, -2.14, -2.35, 1.06});
    request_right_arm.setGoalConfiguration(cob4->getScratchState());

    request_right_arm.setConfig("RRTConnect");

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request_right_arm.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Create a trajectory object for better manipulation.
    auto right_arm_trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    right_arm_trajectory->toYAMLFile("cob4_right_arm_path.yml");

    // Create a motion planning request with a joint position goal for the left arm.
    MotionRequestBuilder request_left_arm(planner, LEFT_ARM);
    request_left_arm.setStartConfiguration(cob4->getScratchState());

    // Create a motion planning request with a pose goal to fold the left arm.
    cob4->setGroupState(LEFT_ARM, {-1.14, -1.50, 0.34, -1.50, 0.43, -1.56, -1.20});
    request_left_arm.setGoalConfiguration(cob4->getScratchState());

    request_left_arm.setConfig("RRTConnect");

    // Do motion planning!
    res = planner->plan(scene, request_left_arm.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Create a trajectory object for better manipulation.
    auto left_arm_trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    left_arm_trajectory->toYAMLFile("cob4_left_arm_path.yml");

    return 0;
}
