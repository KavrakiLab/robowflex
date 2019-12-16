/* Author: Zachary Kingston */

#include <random>

#include <boost/range/combine.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/robot_state/conversions.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/io/visualization.h>

using namespace robowflex;

namespace
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    Eigen::Vector4d getRandomColor()
    {
        std::uniform_real_distribution<> dis(0.2, 0.7);
        return {dis(gen), dis(gen), dis(gen), 1.};
    }
};  // namespace

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
    state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("state",1);
    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("scene", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100);\
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

void IO::RVIZHelper::visualizeState(const std::vector<double> &state)
{
    moveit_msgs::DisplayRobotState out;
    if (state_pub_.getNumSubscribers() < 1)
    {
        ROS_INFO("Waiting for State subscribers...");

        ros::WallDuration pause(0.1);
        while (state_pub_.getNumSubscribers() < 1)
            pause.sleep();
    }
    out.state.joint_state.name = robot_->getJointNames();
    out.state.joint_state.position = state;
    state_pub_.publish(out);
}

void IO::RVIZHelper::fillMarker(visualization_msgs::Marker &marker, const std::string &base_frame,
                                const RobotPose &pose, const Eigen::Vector4d &color,
                                const Eigen::Vector3d &scale) const
{
    marker.header.frame_id = base_frame;
    marker.frame_locked = true;

    marker.header.stamp = ros::Time().now();
    marker.ns = "/robowflex";
    marker.id = markers_.size();

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = TF::poseEigenToMsg(pose);
    marker.scale = TF::vectorEigenToMsg(scale);

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
}

void IO::RVIZHelper::addArrowMarker(const std::string &name, const std::string &base_frame,
                                    const RobotPose &pose, const Eigen::Vector4d &color,
                                    const Eigen::Vector3d &scale)
{
    visualization_msgs::Marker marker;
    fillMarker(marker, base_frame, pose, color, scale);

    marker.type = visualization_msgs::Marker::ARROW;

    markers_.emplace(name, marker);
}

void IO::RVIZHelper::addTextMarker(const std::string &name, const std::string &text,
                                   const std::string &base_frame, const RobotPose &pose, double height,
                                   const Eigen::Vector4d &color)
{
    visualization_msgs::Marker marker;
    fillMarker(marker, base_frame, pose, color, {0, 0, height});

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = text;

    markers_.emplace(name, marker);
}

void IO::RVIZHelper::addGeometryMarker(const std::string &name, const GeometryConstPtr &geometry,
                                       const std::string &base_frame, const RobotPose &pose,
                                       const Eigen::Vector4d &color)
{
    visualization_msgs::Marker marker;

    auto scale = geometry->getDimensions();
    switch (geometry->getType())
    {
        case Geometry::ShapeType::BOX:
            marker.type = visualization_msgs::Marker::CUBE;
            break;
        case Geometry::ShapeType::SPHERE:
            marker.type = visualization_msgs::Marker::SPHERE;
            scale[1] = scale[2] = 2 * scale[0];  // Copy radius to other dimensions
            break;
        case Geometry::ShapeType::CYLINDER:
            marker.type = visualization_msgs::Marker::CYLINDER;
            {
                auto scale2 = scale;
                scale[0] = scale[1] = 2 * scale2[0];  // Copy radius to first two (x & y)
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

    fillMarker(marker, base_frame, pose, color, scale);

    markers_.emplace(name, marker);
}

void IO::RVIZHelper::addGoalMarker(const std::string &name, const MotionRequestBuilder &request)
{
    const auto &goals = request.getRequestConst().goal_constraints;
    const auto &base_frame = "map";  // Markers must be placed in "map" (default RViz frame)

    // Iterate over each goal (an "or"-ing together of different constraints)
    for (const auto &goal : goals)
    {
        auto color = getRandomColor();  // Use the same color for all elements of this goal
        color[3] = 0.7;                 // Make slightly transparent

        for (const auto &pg : goal.position_constraints)
        {
            const auto &pname = pg.link_name;

            // Get global transform of position constraint
            RobotPose pose = robot_->getLinkTF(pg.header.frame_id);
            pose.translate(TF::vectorMsgToEigen(pg.target_point_offset));

            // Iterate over all position primitives and their poses
            for (const auto &primitive :
                 boost::combine(pg.constraint_region.primitives, pg.constraint_region.primitive_poses))
            {
                shape_msgs::SolidPrimitive solid;
                geometry_msgs::Pose solid_pose;
                boost::tie(solid, solid_pose) = primitive;

                // Compute transform of bounding volume
                auto frame = pose * TF::poseMsgToEigen(solid_pose);

                // Add geometry marker associated with this solid primitive
                addGeometryMarker(name, Geometry::makeSolidPrimitive(solid), base_frame, frame, color);

                // Iterate over all orientation constraints for the same link as the position constraint
                for (const auto &og : goal.orientation_constraints)
                {
                    const auto &oname = og.link_name;
                    if (oname != pname)
                        continue;

                    auto q = TF::quaternionMsgToEigen(og.orientation);

                    // Arrow display frame.
                    RobotPose qframe = RobotPose::Identity();
                    qframe.translate(frame.translation());  // Place arrows at the origin of the position
                                                            // volume

                    Eigen::Vector3d scale = {0.1, 0.008, 0.003};  // A nice default size of arrow

                    // Display primary orientation slightly larger
                    addArrowMarker(name, base_frame, qframe * q, color, 1.5 * scale);

                    // Zip together tolerances and axes, and iterate over them to display orientation bounds
                    const auto tolerances = {og.absolute_x_axis_tolerance,  //
                                             og.absolute_y_axis_tolerance,  //
                                             og.absolute_z_axis_tolerance};
                    const auto axes = {Eigen::Vector3d::UnitX(),  //
                                       Eigen::Vector3d::UnitY(),  //
                                       Eigen::Vector3d::UnitZ()};
                    for (const auto &angles : boost::combine(tolerances, axes))
                    {
                        double value;
                        Eigen::Vector3d axis;
                        boost::tie(value, axis) = angles;

                        // Show boundaries of tolerances as smaller arrows.
                        auto q1 = TF::offsetOrientation(q, axis, value);
                        addArrowMarker(name, base_frame, qframe * q1, color, scale);

                        auto q2 = TF::offsetOrientation(q, axis, -value);
                        addArrowMarker(name, base_frame, qframe * q2, color, scale);
                    }
                }
            }

            // TODO: Implement
            // for (const auto &mesh :
            //      boost::combine(pg.constraint_region.meshes, pg.constraint_region.mesh_poses))
            // {
            // }
        }
    }
}

void IO::RVIZHelper::removeMarker(const std::string &name)
{
    auto markers = markers_.equal_range(name);

    for (auto it = markers.first; it != markers.second; ++it)
        it->second.action = visualization_msgs::Marker::DELETE;
}

void IO::RVIZHelper::addMarker(float x, float y, float z)
{
    visualization_msgs::Marker marker;
    const std::string &base_frame = "map";

    RobotPose pose = RobotPose::Identity();
    pose *= Eigen::Translation3d(x, y, z);

    Eigen::Vector3d scale = {0.5, 0.5, 0.5};
    auto color = getRandomColor();

    fillMarker(marker, base_frame, pose, color, scale);

    marker.type = visualization_msgs::Marker::SPHERE;

    markers_.emplace("", marker);
}

void IO::RVIZHelper::removeScene()
{
    updateScene(nullptr);
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

    moveit_msgs::PlanningScene toPub;
    if (scene != nullptr)
    {
        toPub = scene->getMessage();
        toPub.is_diff = true;
    }

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
