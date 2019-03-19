/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
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

    // Opens the Fetch's gripper.
    fetch->openGripper();

    // Dump the geometry information for visualization.
    fetch->dumpGeometry("fetch.yml");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch_goal_regions_scenario1.yml");

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    std::string package_path = ros::package::getPath("multimodal_common_bringup");
    package_path = package_path + std::string("/parameters/moveit/ompl_planning.yaml");
    planner->initialize(OMPL::Settings(), package_path);

    // Create a motion planning request with a pose goal.
    MotionRequestBuilderPtr request(new MotionRequestBuilder(planner, GROUP));
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request->setStartConfiguration(fetch->getScratchState());

    MotionRequestBuilderPtr request2(new MotionRequestBuilder(planner, GROUP));
    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    request2->setStartConfiguration(fetch->getScratchState());

    //
    moveit_msgs::GoalRegion goal_region;
    geometry_msgs::PoseStamped target_pose_grasp;
    std::vector<geometry_msgs::PoseStamped> goal_poses;
    std::vector<moveit_msgs::GoalRegion> goal_regions;

    // Cube1
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(Eigen::Vector3d{0.0, 0.6, 0.92});
    Eigen::Quaterniond orn{0.5, -0.5, 0.5, 0.5};
    auto region = Geometry::makeSphere(0.05);

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
    request->setAllowedPlanningTime(120.0);

    request2->setPoseRegionTargets(goal_poses, goal_regions, "wrist_roll_link");
    request2->setConfig("RRTGoalRegConskConfigDefault");
    request2->setAllowedPlanningTime(120.0);

    //    // Create a motion planning request with a pose goal.
    //    MotionRequestBuilderPtr request2(new MotionRequestBuilder(planner, GROUP));
    //    fetch->setGroupState(GROUP, {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0});  // Stow
    //    request2->setStartConfiguration(fetch->getScratchState());
    //
    //    fetch->setGroupState(GROUP, {0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});  // Unfurl
    //    request2->setGoalConfiguration(fetch->getScratchState());
    //
    //    request2->setConfig("RRTConnect");

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Benchmarker benchmark;
    //    benchmark.addBenchmarkingRequest("joint", scene, planner, joint_request);
    //    benchmark.addBenchmarkingRequest("pose", scene, planner, pose_request);
    benchmark.addBenchmarkingRequest("RRTRewardPenalty", scene, planner, request2);
    benchmark.addBenchmarkingRequest("RRTConsecutive", scene, planner, request);
    // benchmark.Options()

    // Output results to an OMPL benchmarking file.
    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_fetch_test/")},
                        Benchmarker::Options(100));

    return 0;
}
