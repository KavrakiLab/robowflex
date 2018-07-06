/* Author: Zachary Kingston */

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_state/conversions.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/io/visualization.h>

using namespace robowflex;

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
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
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

            // do nothing
            /*
            if(out.trajectory == NULL) {
                std::cout<<"Trajectory is null"<<std::endl;
            }
            */
        }
    }

    trajectory_pub_.publish(out);
}

void IO::RVIZHelper::addMarker(float x, float y, float z)
{
    std::cout<<"Adding the marker"<<std::endl;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time().now();
    marker.ns = "Sphere";
    marker.id = 8;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker_pub_.publish(marker);
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
