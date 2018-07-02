/* Author: Zachary Kingston */

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_state/conversions.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/io/visualization.h>

using namespace robowflex;

///
/// IO::RVIZHelper
///

IO::RVIZHelper::RVIZHelper(const RobotConstPtr &robot, const std::string &name)
  : robot_(robot), nh_("/" + name)
{
    std::string description;
    robot_->getHandlerConst().getParam(Robot::ROBOT_DESCRIPTION, description);
    std::string semantic;
    robot_->getHandlerConst().getParam(Robot::ROBOT_DESCRIPTION + Robot::ROBOT_SEMANTIC, semantic);

    nh_.setParam(Robot::ROBOT_DESCRIPTION, description);
    nh_.setParam(Robot::ROBOT_DESCRIPTION + Robot::ROBOT_SEMANTIC, semantic);

    trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 0);
    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("scene", 0);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 0);
}

void IO::RVIZHelper::updateTrajectory(const planning_interface::MotionPlanResponse &response)
{
    moveit_msgs::DisplayTrajectory out;

    moveit_msgs::RobotTrajectory msg;
    response.trajectory_->getRobotTrajectoryMsg(msg);

    out.model_id = robot_->getModelName();
    out.trajectory.push_back(msg);
    moveit::core::robotStateToRobotStateMsg(response.trajectory_->getFirstWayPoint(), out.trajectory_start);

    trajectory_pub_.publish(out);
}

void IO::RVIZHelper::updateTrajectories(const std::vector<planning_interface::MotionPlanResponse> &responses)
{
    moveit_msgs::DisplayTrajectory out;
    out.model_id = robot_->getModelName();

    bool set = false;
    for (const auto &response : responses)
    {
        if (!set)
        {
            moveit::core::robotStateToRobotStateMsg(response.trajectory_->getFirstWayPoint(),
                                                    out.trajectory_start);
            set = true;
        }

        if(response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            moveit_msgs::RobotTrajectory msg;
            response.trajectory_->getRobotTrajectoryMsg(msg);
            out.trajectory.push_back(msg);
        }
        else {
            moveit_msgs::RobotTrajectory msg;
            //response.trajectory_->getRobotTrajectoryMsg(msg);
            //Not sure what to do for a failed motion
            //Use a marker to indicate the final goal?
            visualization_msgs::Marker marker;
            marker.header.frame_id = "goal_marker";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = 97;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 10;
            marker.scale.y = 10.1;
            marker.scale.z = 10.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            visualization_msgs::MarkerArray arr;
            arr.markers.push_back(marker);
            marker_pub_.publish(arr); 

            //out.trajectory.push_back(msg);
        }

    }

    trajectory_pub_.publish(out);
}

void IO::RVIZHelper::updateScene(const SceneConstPtr &scene)
{
    scene_pub_.publish(scene->getMessage());
}

void IO::RVIZHelper::updateMarkers()
{
    visualization_msgs::MarkerArray msg;

    std::vector<std::string> remove;
    for (auto &marker : markers_)
    {
        msg.markers.push_back(marker.second);

        if (marker.second.action == visualization_msgs::Marker::ADD)
            marker.second.action = visualization_msgs::Marker::MODIFY;
        else if (marker.second.action == visualization_msgs::Marker::DELETE)
            remove.push_back(marker.first);
    }

    marker_pub_.publish(msg);

    for (auto &marker : remove)
        markers_.erase(markers_.find(marker));
}
