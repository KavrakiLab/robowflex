/* Author: Zachary Kingston */

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/robot_state/conversions.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/geometry.h>
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

    trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);
    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("scene", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100);
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

        if (response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            moveit_msgs::RobotTrajectory msg;
            response.trajectory_->getRobotTrajectoryMsg(msg);
            out.trajectory.push_back(msg);
        }
    }

    if (trajectory_pub_.getNumSubscribers() < 1)
    {
        ROS_INFO("Waiting for Trajectory subscribers...");

        ros::WallDuration pause(0.1);
        while (trajectory_pub_.getNumSubscribers() < 1)
            pause.sleep();
    }

    trajectory_pub_.publish(out);
}

void IO::RVIZHelper::addGeometryMarker(const std::string &name, const GeometryConstPtr &geometry,
                                       const std::string &base_frame, const Eigen::Affine3d &pose,
                                       const Eigen::Vector4d &color)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = base_frame;
    if (base_frame != "world")
        marker.frame_locked = true;

    marker.header.stamp = ros::Time().now();
    marker.ns = "/robowflex";
    marker.id = markers_.size();

    auto scale = geometry->getDimensions();
    switch (geometry->getType())
    {
        case Geometry::ShapeType::BOX:
            marker.type = visualization_msgs::Marker::CUBE;
            break;
        case Geometry::ShapeType::SPHERE:
            marker.type = visualization_msgs::Marker::SPHERE;
            scale[1] = scale[2] = scale[0];  // Copy radius to other dimensions
            break;
        case Geometry::ShapeType::CYLINDER:
            marker.type = visualization_msgs::Marker::CYLINDER;
            {
                auto scale2 = scale;
                scale[0] = scale[1] = scale2[0];  // Copy radius to first two (x & y)
                scale[2] = scale2[1];
            }
            break;
        case Geometry::ShapeType::MESH:
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = geometry->getResource();
            marker.mesh_use_embedded_materials = true;
            break;
        default:
            ROS_ERROR("Unsupported geometry for marker.");
            return;
    };

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = TF::poseEigenToMsg(pose);
    marker.scale = TF::vectorEigenToMsg(scale);

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    markers_.emplace(name, marker);
}

// void IO::RVIZHelper::addGoalMarker(const MotionRequestBuilder &request)
// {

// }

void IO::RVIZHelper::removeMarker(const std::string &name)
{
    auto marker = markers_.find(name);
    if (marker != markers_.end())
        marker->second.action = visualization_msgs::Marker::DELETE;
}

void IO::RVIZHelper::addMarker(float x, float y, float z)
{
}

void IO::RVIZHelper::updateScene(const SceneConstPtr &scene)
{
    if (scene_pub_.getNumSubscribers() < 1)
    {
        ROS_INFO("Waiting for Scene subscribers...");

        ros::WallDuration pause(0.1);
        while (scene_pub_.getNumSubscribers() < 1)
            pause.sleep();
    }

    moveit_msgs::PlanningScene toPub = scene->getMessage();
    toPub.is_diff = true;
    scene_pub_.publish(toPub);
}

void IO::RVIZHelper::updateMarkers()
{
    visualization_msgs::MarkerArray msg;

    std::vector<std::string> remove;
    for (auto &marker : markers_)
    {
        msg.markers.emplace_back(marker.second);

        if (marker.second.action == visualization_msgs::Marker::ADD)
            marker.second.action = visualization_msgs::Marker::MODIFY;

        else if (marker.second.action == visualization_msgs::Marker::DELETE)
            remove.push_back(marker.first);
    }

    if (marker_pub_.getNumSubscribers() < 1)
    {
        ROS_INFO("Waiting for MarkerArray subscribers...");

        ros::WallDuration pause(0.1);
        while (marker_pub_.getNumSubscribers() < 1)
            pause.sleep();
    }

    marker_pub_.publish(msg);

    for (auto &marker : remove)
        markers_.erase(markers_.find(marker));
}
