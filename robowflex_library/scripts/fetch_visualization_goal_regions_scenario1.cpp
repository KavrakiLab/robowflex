/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Sets the Fetch's base pose.
    // fetch->setBasePose(1.5, 0.0, 0.0);

    // Sets the Fetch's head pose to look at a point.
    // fetch->pointHead({2, 1, 1.5});

    // Opens the Fetch's gripper.
    fetch->openGripper();

    // Dump the geometry information for visualization.
    fetch->dumpGeometry("fetch.yml");

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(fetch);

    ROS_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch_goal_regions_scenario1.yml");

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize(OMPL::Settings(), "/home/juandhv/ros/melodic/system/src/multimodal_common_bringup/"
                                          "parameters/moveit/ompl_planning.yaml");

    // Create a motion planning request with a pose goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, GROUP));
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(fetch->getScratchState());

    //
    moveit_msgs::GoalRegion goal_region;
    geometry_msgs::PoseStamped target_pose_grasp;
    std::vector<geometry_msgs::PoseStamped> goal_poses;
    std::vector<moveit_msgs::GoalRegion> goal_regions;

    //----------------------------------------

    // Grasp Can1
    target_pose_grasp.header.frame_id = "world";
    target_pose_grasp.pose.position.x = 0.65;
    target_pose_grasp.pose.position.y = -0.185;
    target_pose_grasp.pose.position.z = 0.95;
    target_pose_grasp.pose.orientation.x = -0.5;
    target_pose_grasp.pose.orientation.y = 0.5;
    target_pose_grasp.pose.orientation.z = 0.5;
    target_pose_grasp.pose.orientation.w = 0.5;

    goal_region.header.frame_id = "world";
    goal_region.x.min = target_pose_grasp.pose.position.x;
    goal_region.x.max = target_pose_grasp.pose.position.x;
    goal_region.x.free_value = false;
    goal_region.y.min = target_pose_grasp.pose.position.y;
    goal_region.y.max = target_pose_grasp.pose.position.y;
    goal_region.y.free_value = false;
    goal_region.z.min = target_pose_grasp.pose.position.z;
    goal_region.z.max = target_pose_grasp.pose.position.z;
    goal_region.z.free_value = false;
    goal_region.roll.min = goal_region.roll.max = -1.5707963267948968;
    goal_region.roll.free_value = false;
    goal_region.pitch.min = goal_region.pitch.max = 1.5707963267948963;
    goal_region.pitch.free_value = false;
    goal_region.yaw.min = goal_region.yaw.max = 0.0;
    goal_region.yaw.free_value = true;

    goal_poses.push_back(target_pose_grasp);
    goal_regions.push_back(goal_region);

    // Grasp Can2
    target_pose_grasp.header.frame_id = "world";
    target_pose_grasp.pose.position.x = 0.65;
    target_pose_grasp.pose.position.y = 0.185;
    target_pose_grasp.pose.position.z = 0.95;
    target_pose_grasp.pose.orientation.x = -0.5;
    target_pose_grasp.pose.orientation.y = 0.5;
    target_pose_grasp.pose.orientation.z = 0.5;
    target_pose_grasp.pose.orientation.w = 0.5;

    goal_region.header.frame_id = "world";
    goal_region.x.min = target_pose_grasp.pose.position.x;
    goal_region.x.max = target_pose_grasp.pose.position.x;
    goal_region.x.free_value = false;
    goal_region.y.min = target_pose_grasp.pose.position.y;
    goal_region.y.max = target_pose_grasp.pose.position.y;
    goal_region.y.free_value = false;
    goal_region.z.min = target_pose_grasp.pose.position.z;
    goal_region.z.max = target_pose_grasp.pose.position.z;
    goal_region.z.free_value = false;
    goal_region.roll.min = goal_region.roll.max = -1.5707963267948968;
    goal_region.roll.free_value = false;
    goal_region.pitch.min = goal_region.pitch.max = 1.5707963267948963;
    goal_region.pitch.free_value = false;
    goal_region.yaw.min = goal_region.yaw.max = 0.0;
    goal_region.yaw.free_value = true;

    goal_poses.push_back(target_pose_grasp);
    goal_regions.push_back(goal_region);

    // Grasp Can3
    target_pose_grasp.header.frame_id = "world";
    target_pose_grasp.pose.position.x = 0.4;
    target_pose_grasp.pose.position.y = 0.5;
    target_pose_grasp.pose.position.z = 0.95;
    target_pose_grasp.pose.orientation.x = -0.5;
    target_pose_grasp.pose.orientation.y = 0.5;
    target_pose_grasp.pose.orientation.z = 0.5;
    target_pose_grasp.pose.orientation.w = 0.5;

    goal_region.header.frame_id = "world";
    goal_region.x.min = target_pose_grasp.pose.position.x;
    goal_region.x.max = target_pose_grasp.pose.position.x;
    goal_region.x.free_value = false;
    goal_region.y.min = target_pose_grasp.pose.position.y;
    goal_region.y.max = target_pose_grasp.pose.position.y;
    goal_region.y.free_value = false;
    goal_region.z.min = target_pose_grasp.pose.position.z;
    goal_region.z.max = target_pose_grasp.pose.position.z;
    goal_region.z.free_value = false;
    goal_region.roll.min = goal_region.roll.max = -1.5707963267948968;
    goal_region.roll.free_value = false;
    goal_region.pitch.min = goal_region.pitch.max = 1.5707963267948963;
    goal_region.pitch.free_value = false;
    goal_region.yaw.min = goal_region.yaw.max = 0.0;
    goal_region.yaw.free_value = true;

    goal_poses.push_back(target_pose_grasp);
    goal_regions.push_back(goal_region);

    // Grasp Can4
    target_pose_grasp.header.frame_id = "world";
    target_pose_grasp.pose.position.x = 0.3;
    target_pose_grasp.pose.position.y = -0.7;
    target_pose_grasp.pose.position.z = 0.95;
    target_pose_grasp.pose.orientation.x = -0.5;
    target_pose_grasp.pose.orientation.y = 0.5;
    target_pose_grasp.pose.orientation.z = 0.5;
    target_pose_grasp.pose.orientation.w = 0.5;

    goal_region.header.frame_id = "world";
    goal_region.x.min = target_pose_grasp.pose.position.x;
    goal_region.x.max = target_pose_grasp.pose.position.x;
    goal_region.x.free_value = false;
    goal_region.y.min = target_pose_grasp.pose.position.y;
    goal_region.y.max = target_pose_grasp.pose.position.y;
    goal_region.y.free_value = false;
    goal_region.z.min = target_pose_grasp.pose.position.z;
    goal_region.z.max = target_pose_grasp.pose.position.z;
    goal_region.z.free_value = false;
    goal_region.roll.min = goal_region.roll.max = -1.5707963267948968;
    goal_region.roll.free_value = false;
    goal_region.pitch.min = goal_region.pitch.max = 1.5707963267948963;
    goal_region.pitch.free_value = false;
    goal_region.yaw.min = goal_region.yaw.max = 0.0;
    goal_region.yaw.free_value = true;

    goal_poses.push_back(target_pose_grasp);
    goal_regions.push_back(goal_region);

    //--

    request->setPoseRegionTargets(goal_poses, goal_regions, "wrist_roll_link");
    request->setConfig("RRTGoalRegionk");
    // request->setConfig("RRTGoalRegConskConfigDefault");
    // request->setConfig("RRTModkConfigDefault");
    request->setAllowedPlanningTime(60.0);

    ROS_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    ROS_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    rviz.removeMarker("goal");
    rviz.updateMarkers();

    rviz.removeScene();

    ROS_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
