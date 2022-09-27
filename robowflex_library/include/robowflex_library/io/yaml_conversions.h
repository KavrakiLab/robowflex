/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_YAML_
#define ROBOWFLEX_YAML_

#include <ros/time.h>
#include <ros/duration.h>

#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/MultiDOFJointState.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <object_recognition_msgs/ObjectType.h>

#include <moveit_msgs/LinkPadding.h>
#include <moveit_msgs/LinkScale.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit_msgs/BoundingVolume.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/VisibilityConstraint.h>
#include <moveit_msgs/TrajectoryConstraints.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit_msgs/MoveGroupResult.h>

#include <yaml-cpp/yaml.h>

#include <robowflex_library/macros.h>

/** \cond IGNORE */
namespace YAML
{
#if ROBOWFLEX_AT_LEAST_KINETIC
#else
    template <>
    struct convert<signed char>
    {
        static Node encode(const signed char &rhs);
        static bool decode(const Node &node, signed char &rhs);
    };
#endif

    template <>
    struct convert<moveit_msgs::PlanningScene>
    {
        static Node encode(const moveit_msgs::PlanningScene &rhs);
        static bool decode(const Node &node, moveit_msgs::PlanningScene &rhs);
    };

    template <>
    struct convert<moveit_msgs::RobotState>
    {
        static Node encode(const moveit_msgs::RobotState &rhs);
        static bool decode(const Node &node, moveit_msgs::RobotState &rhs);
    };

    template <>
    struct convert<geometry_msgs::TransformStamped>
    {
        static Node encode(const geometry_msgs::TransformStamped &rhs);
        static bool decode(const Node &node, geometry_msgs::TransformStamped &rhs);
    };

    template <>
    struct convert<std_msgs::Header>
    {
        static Node encode(const std_msgs::Header &rhs);
        static bool decode(const Node &node, std_msgs::Header &rhs);
    };

    template <>
    struct convert<geometry_msgs::Pose>
    {
        static Node encode(const geometry_msgs::Pose &rhs);
        static bool decode(const Node &node, geometry_msgs::Pose &rhs);
    };

    template <>
    struct convert<geometry_msgs::PoseStamped>
    {
        static Node encode(const geometry_msgs::PoseStamped &rhs);
        static bool decode(const Node &node, geometry_msgs::PoseStamped &rhs);
    };

    template <>
    struct convert<geometry_msgs::Transform>
    {
        static Node encode(const geometry_msgs::Transform &rhs);
        static bool decode(const Node &node, geometry_msgs::Transform &rhs);
    };

    template <>
    struct convert<geometry_msgs::Vector3>
    {
        static Node encode(const geometry_msgs::Vector3 &rhs);
        static bool decode(const Node &node, geometry_msgs::Vector3 &rhs);
    };

    template <>
    struct convert<geometry_msgs::Point>
    {
        static Node encode(const geometry_msgs::Point &rhs);
        static bool decode(const Node &node, geometry_msgs::Point &rhs);
    };

    template <>
    struct convert<geometry_msgs::Quaternion>
    {
        static Node encode(const geometry_msgs::Quaternion &rhs);
        static bool decode(const Node &node, geometry_msgs::Quaternion &rhs);
    };

    template <>
    struct convert<geometry_msgs::Twist>
    {
        static Node encode(const geometry_msgs::Twist &rhs);
        static bool decode(const Node &node, geometry_msgs::Twist &rhs);
    };

    template <>
    struct convert<geometry_msgs::Wrench>
    {
        static Node encode(const geometry_msgs::Wrench &rhs);
        static bool decode(const Node &node, geometry_msgs::Wrench &rhs);
    };

    template <>
    struct convert<sensor_msgs::JointState>
    {
        static Node encode(const sensor_msgs::JointState &rhs);
        static bool decode(const Node &node, sensor_msgs::JointState &rhs);
    };

    template <>
    struct convert<sensor_msgs::MultiDOFJointState>
    {
        static Node encode(const sensor_msgs::MultiDOFJointState &rhs);
        static bool decode(const Node &node, sensor_msgs::MultiDOFJointState &rhs);
    };

    template <>
    struct convert<moveit_msgs::AttachedCollisionObject>
    {
        static Node encode(const moveit_msgs::AttachedCollisionObject &rhs);
        static bool decode(const Node &node, moveit_msgs::AttachedCollisionObject &rhs);
    };

    template <>
    struct convert<trajectory_msgs::JointTrajectory>
    {
        static Node encode(const trajectory_msgs::JointTrajectory &rhs);
        static bool decode(const Node &node, trajectory_msgs::JointTrajectory &rhs);
    };

    template <>
    struct convert<trajectory_msgs::JointTrajectoryPoint>
    {
        static Node encode(const trajectory_msgs::JointTrajectoryPoint &rhs);
        static bool decode(const Node &node, trajectory_msgs::JointTrajectoryPoint &rhs);
    };

    template <>
    struct convert<moveit_msgs::CollisionObject>
    {
        static Node encode(const moveit_msgs::CollisionObject &rhs);
        static bool decode(const Node &node, moveit_msgs::CollisionObject &rhs);
    };

    template <>
    struct convert<object_recognition_msgs::ObjectType>
    {
        static Node encode(const object_recognition_msgs::ObjectType &rhs);
        static bool decode(const Node &node, object_recognition_msgs::ObjectType &rhs);
    };

    template <>
    struct convert<moveit_msgs::LinkPadding>
    {
        static Node encode(const moveit_msgs::LinkPadding &rhs);
        static bool decode(const Node &node, moveit_msgs::LinkPadding &rhs);
    };

    template <>
    struct convert<moveit_msgs::LinkScale>
    {
        static Node encode(const moveit_msgs::LinkScale &rhs);
        static bool decode(const Node &node, moveit_msgs::LinkScale &rhs);
    };

    template <>
    struct convert<moveit_msgs::AllowedCollisionMatrix>
    {
        static Node encode(const moveit_msgs::AllowedCollisionMatrix &rhs);
        static bool decode(const Node &node, moveit_msgs::AllowedCollisionMatrix &rhs);
    };

    template <>
    struct convert<moveit_msgs::AllowedCollisionEntry>
    {
        static Node encode(const moveit_msgs::AllowedCollisionEntry &rhs);
        static bool decode(const Node &node, moveit_msgs::AllowedCollisionEntry &rhs);
    };

    template <>
    struct convert<moveit_msgs::PlanningSceneWorld>
    {
        static Node encode(const moveit_msgs::PlanningSceneWorld &rhs);
        static bool decode(const Node &node, moveit_msgs::PlanningSceneWorld &rhs);
    };

    template <>
    struct convert<moveit_msgs::ObjectColor>
    {
        static Node encode(const moveit_msgs::ObjectColor &rhs);
        static bool decode(const Node &node, moveit_msgs::ObjectColor &rhs);
    };

    template <>
    struct convert<std_msgs::ColorRGBA>
    {
        static Node encode(const std_msgs::ColorRGBA &rhs);
        static bool decode(const Node &node, std_msgs::ColorRGBA &rhs);
    };

    template <>
    struct convert<octomap_msgs::Octomap>
    {
        static Node encode(const octomap_msgs::Octomap &rhs);
        static bool decode(const Node &node, octomap_msgs::Octomap &rhs);
    };

    template <>
    struct convert<octomap_msgs::OctomapWithPose>
    {
        static Node encode(const octomap_msgs::OctomapWithPose &rhs);
        static bool decode(const Node &node, octomap_msgs::OctomapWithPose &rhs);
    };

    template <>
    struct convert<ros::Time>
    {
        static Node encode(const ros::Time &rhs);
        static bool decode(const Node &node, ros::Time &rhs);
    };

    template <>
    struct convert<ros::Duration>
    {
        static Node encode(const ros::Duration &rhs);
        static bool decode(const Node &node, ros::Duration &rhs);
    };

    template <>
    struct convert<shape_msgs::SolidPrimitive>
    {
        static Node encode(const shape_msgs::SolidPrimitive &rhs);
        static bool decode(const Node &node, shape_msgs::SolidPrimitive &rhs);
    };

    template <>
    struct convert<shape_msgs::Mesh>
    {
        static Node encode(const shape_msgs::Mesh &rhs);
        static bool decode(const Node &node, shape_msgs::Mesh &rhs);
    };

    template <>
    struct convert<shape_msgs::MeshTriangle>
    {
        static Node encode(const shape_msgs::MeshTriangle &rhs);
        static bool decode(const Node &node, shape_msgs::MeshTriangle &rhs);
    };

    template <>
    struct convert<shape_msgs::Plane>
    {
        static Node encode(const shape_msgs::Plane &rhs);
        static bool decode(const Node &node, shape_msgs::Plane &rhs);
    };

    template <>
    struct convert<moveit_msgs::WorkspaceParameters>
    {
        static Node encode(const moveit_msgs::WorkspaceParameters &rhs);
        static bool decode(const Node &node, moveit_msgs::WorkspaceParameters &rhs);
    };

    template <>
    struct convert<moveit_msgs::Constraints>
    {
        static Node encode(const moveit_msgs::Constraints &rhs);
        static bool decode(const Node &node, moveit_msgs::Constraints &rhs);
    };

    template <>
    struct convert<moveit_msgs::JointConstraint>
    {
        static Node encode(const moveit_msgs::JointConstraint &rhs);
        static bool decode(const Node &node, moveit_msgs::JointConstraint &rhs);
    };

    template <>
    struct convert<moveit_msgs::PositionConstraint>
    {
        static Node encode(const moveit_msgs::PositionConstraint &rhs);
        static bool decode(const Node &node, moveit_msgs::PositionConstraint &rhs);
    };

    template <>
    struct convert<moveit_msgs::OrientationConstraint>
    {
        static Node encode(const moveit_msgs::OrientationConstraint &rhs);
        static bool decode(const Node &node, moveit_msgs::OrientationConstraint &rhs);
    };

    template <>
    struct convert<moveit_msgs::VisibilityConstraint>
    {
        static Node encode(const moveit_msgs::VisibilityConstraint &rhs);
        static bool decode(const Node &node, moveit_msgs::VisibilityConstraint &rhs);
    };

    template <>
    struct convert<moveit_msgs::TrajectoryConstraints>
    {
        static Node encode(const moveit_msgs::TrajectoryConstraints &rhs);
        static bool decode(const Node &node, moveit_msgs::TrajectoryConstraints &rhs);
    };

    template <>
    struct convert<moveit_msgs::BoundingVolume>
    {
        static Node encode(const moveit_msgs::BoundingVolume &rhs);
        static bool decode(const Node &node, moveit_msgs::BoundingVolume &rhs);
    };

    template <>
    struct convert<moveit_msgs::MotionPlanRequest>
    {
        static Node encode(const moveit_msgs::MotionPlanRequest &rhs);
        static bool decode(const Node &node, moveit_msgs::MotionPlanRequest &rhs);
    };

    template <>
    struct convert<trajectory_msgs::MultiDOFJointTrajectory>
    {
        static Node encode(const trajectory_msgs::MultiDOFJointTrajectory &rhs);
        static bool decode(const Node &node, trajectory_msgs::MultiDOFJointTrajectory &rhs);
    };

    template <>
    struct convert<trajectory_msgs::MultiDOFJointTrajectoryPoint>
    {
        static Node encode(const trajectory_msgs::MultiDOFJointTrajectoryPoint &rhs);
        static bool decode(const Node &node, trajectory_msgs::MultiDOFJointTrajectoryPoint &rhs);
    };

    template <>
    struct convert<moveit_msgs::RobotTrajectory>
    {
        static Node encode(const moveit_msgs::RobotTrajectory &rhs);
        static bool decode(const Node &node, moveit_msgs::RobotTrajectory &rhs);
    };
}  // namespace YAML
/* \endcond */

#endif
