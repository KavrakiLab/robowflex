#include <algorithm>
#include <string>

#include "robowflex.h"

using namespace robowflex;

namespace
{
    const std::string boolToString(bool b)
    {
        return b ? "true" : "false";
    }

    bool nodeToBool(const YAML::Node &n)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return (s == "true") ? true : false;
    }
}  // namespace

namespace YAML
{
    Node convert<moveit_msgs::PlanningScene>::encode(const moveit_msgs::PlanningScene &rhs)
    {
        Node node;
        node["name"] = rhs.name;
        node["robot_state"] = rhs.robot_state;
        node["robot_model_name"] = rhs.robot_model_name;
        node["fixed_frame_transforms"] = rhs.fixed_frame_transforms;
        node["allowed_collision_matrix"] = rhs.allowed_collision_matrix;

        // node["link_padding"] = rhs.link_padding;
        // node["link_padding"].SetStyle(YAML::EmitterStyle::Flow);

        // node["link_scale"] = rhs.link_scale;
        // node["link_scale"].SetStyle(YAML::EmitterStyle::Flow);

        // node["object_colors"] = rhs.object_colors;

        node["world"] = rhs.world;
        node["is_diff"] = boolToString(rhs.is_diff);
        return node;
    }

    bool convert<moveit_msgs::PlanningScene>::decode(const Node &node, moveit_msgs::PlanningScene &rhs)
    {
        rhs = moveit_msgs::PlanningScene();

        if (node["name"])
            rhs.name = node["name"].as<std::string>();

        if (node["robot_state"])
            rhs.robot_state = node["robot_state"].as<moveit_msgs::RobotState>();

        if (node["robot_model_name"])
            rhs.robot_model_name = node["robot_model_name"].as<std::string>();

        if (node["fixed_frame_transforms"])
            rhs.fixed_frame_transforms =
                node["fixed_frame_transforms"].as<std::vector<geometry_msgs::TransformStamped>>();

        if (node["allowed_collision_matrix"])
            rhs.allowed_collision_matrix = node["allowed_collision_matrix"].as<moveit_msgs::AllowedCollisionMatrix>();

        if (node["link_padding"])
            rhs.link_padding = node["link_padding"].as<std::vector<moveit_msgs::LinkPadding>>();

        if (node["link_scale"])
            rhs.link_scale = node["link_scale"].as<std::vector<moveit_msgs::LinkScale>>();

        if (node["object_colors"])
            rhs.object_colors = node["object_colors"].as<std::vector<moveit_msgs::ObjectColor>>();

        if (node["world"])
            rhs.world = node["world"].as<moveit_msgs::PlanningSceneWorld>();

        if (node["is_diff"])
            rhs.is_diff = nodeToBool(node["is_diff"]);

        return true;
    }

    Node convert<moveit_msgs::RobotState>::encode(const moveit_msgs::RobotState &rhs)
    {
        Node node;
        node["joint_state"] = rhs.joint_state;
        node["multi_dof_joint_state"] = rhs.multi_dof_joint_state;
        node["attached_collision_objects"] = rhs.attached_collision_objects;
        node["is_diff"] = boolToString(rhs.is_diff);
        return node;
    }

    bool convert<moveit_msgs::RobotState>::decode(const Node &node, moveit_msgs::RobotState &rhs)
    {
        rhs = moveit_msgs::RobotState();

        if (node["joint_state"])
            rhs.joint_state = node["joint_state"].as<sensor_msgs::JointState>();

        if (node["multi_dof_joint_state"])
            rhs.multi_dof_joint_state = node["multi_dof_joint_state"].as<sensor_msgs::MultiDOFJointState>();

        if (node["attached_collision_objects"])
            rhs.attached_collision_objects =
                node["attached_collision_objects"].as<std::vector<moveit_msgs::AttachedCollisionObject>>();

        if (node["is_diff"])
            rhs.is_diff = nodeToBool(node["is_diff"]);

        return true;
    }

    Node convert<geometry_msgs::TransformStamped>::encode(const geometry_msgs::TransformStamped &rhs)
    {
        Node node;
        node["header"] = rhs.header;
        node["child_frame_id"] = rhs.child_frame_id;
        node["transform"] = rhs.transform;
        return node;
    }

    bool convert<geometry_msgs::TransformStamped>::decode(const Node &node, geometry_msgs::TransformStamped &rhs)
    {
        rhs = geometry_msgs::TransformStamped();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();

        if (node["child_frame_id"])
            rhs.child_frame_id = node["child_frame_id"].as<std::string>();

        if (node["transform"])
            rhs.transform = node["transform"].as<geometry_msgs::Transform>();

        return true;
    }

    Node convert<std_msgs::Header>::encode(const std_msgs::Header &rhs)
    {
        Node node;
        node["seq"] = rhs.seq;
        node["stamp"]["sec"] = rhs.stamp.sec;
        node["stamp"]["nsec"] = rhs.stamp.nsec;
        node["frame_id"] = rhs.frame_id;
        return node;
    }

    bool convert<std_msgs::Header>::decode(const Node &node, std_msgs::Header &rhs)
    {
        rhs = std_msgs::Header();

        if (node["seq"])
            rhs.seq = node["seq"].as<int>();

        if (node["stamp"])
        {
            rhs.stamp.sec = node["stamp"]["sec"].as<int>();
            rhs.stamp.nsec = node["stamp"]["nsec"].as<int>();
        }

        if (node["frame_id"])
            rhs.frame_id = node["frame_id"].as<std::string>();

        return true;
    }

    Node convert<geometry_msgs::Transform>::encode(const geometry_msgs::Transform &rhs)
    {
        Node node;
        node["translation"] = rhs.translation;
        node["rotation"] = rhs.rotation;
        return node;
    }

    bool convert<geometry_msgs::Transform>::decode(const Node &node, geometry_msgs::Transform &rhs)
    {
        rhs = geometry_msgs::Transform();

        if (node["translation"])
            rhs.translation = node["translation"].as<geometry_msgs::Vector3>();

        if (node["rotation"])
            rhs.rotation = node["rotation"].as<geometry_msgs::Quaternion>();

        return true;
    }

    Node convert<geometry_msgs::Vector3>::encode(const geometry_msgs::Vector3 &rhs)
    {
        Node node;
        node.SetStyle(YAML::EmitterStyle::Flow);

        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        return node;
    }

    bool convert<geometry_msgs::Vector3>::decode(const Node &node, geometry_msgs::Vector3 &rhs)
    {
        rhs = geometry_msgs::Vector3();

        rhs.x = node[0].as<double>();
        rhs.y = node[1].as<double>();
        rhs.z = node[2].as<double>();
        return true;
    }

    Node convert<geometry_msgs::Quaternion>::encode(const geometry_msgs::Quaternion &rhs)
    {
        Node node;
        node.SetStyle(YAML::EmitterStyle::Flow);

        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        node.push_back(rhs.w);
        return node;
    }

    bool convert<geometry_msgs::Quaternion>::decode(const Node &node, geometry_msgs::Quaternion &rhs)
    {
        rhs = geometry_msgs::Quaternion();

        rhs.x = node[0].as<double>();
        rhs.y = node[1].as<double>();
        rhs.z = node[2].as<double>();
        rhs.w = node[3].as<double>();
        return true;
    }

    Node convert<geometry_msgs::Twist>::encode(const geometry_msgs::Twist &rhs)
    {
        Node node;
        node["linear"] = rhs.linear;
        node["angular"] = rhs.angular;
        return node;
    }

    bool convert<geometry_msgs::Twist>::decode(const Node &node, geometry_msgs::Twist &rhs)
    {
        rhs = geometry_msgs::Twist();

        rhs.linear = node["linear"].as<geometry_msgs::Vector3>();
        rhs.angular = node["angular"].as<geometry_msgs::Vector3>();

        return true;
    }

    Node convert<geometry_msgs::Wrench>::encode(const geometry_msgs::Wrench &rhs)
    {
        Node node;
        node["force"] = rhs.force;
        node["torque"] = rhs.torque;
        return node;
    }

    bool convert<geometry_msgs::Wrench>::decode(const Node &node, geometry_msgs::Wrench &rhs)
    {
        rhs = geometry_msgs::Wrench();

        rhs.force = node["force"].as<geometry_msgs::Vector3>();
        rhs.torque = node["torque"].as<geometry_msgs::Vector3>();

        return true;
    }

    Node convert<sensor_msgs::JointState>::encode(const sensor_msgs::JointState &rhs)
    {
        Node node;
        node["header"] = rhs.header;
        node["name"] = rhs.name;
        node["name"].SetStyle(YAML::EmitterStyle::Flow);
        node["position"] = rhs.position;
        node["position"].SetStyle(YAML::EmitterStyle::Flow);
        node["velocity"] = rhs.velocity;
        node["velocity"].SetStyle(YAML::EmitterStyle::Flow);
        node["effort"] = rhs.effort;
        node["effort"].SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    bool convert<sensor_msgs::JointState>::decode(const Node &node, sensor_msgs::JointState &rhs)
    {
        rhs = sensor_msgs::JointState();
        rhs.header = node["header"].as<std_msgs::Header>();

        if (node["name"])
            rhs.name = node["name"].as<std::vector<std::string>>();

        if (node["position"])
            rhs.position = node["position"].as<std::vector<double>>();

        if (node["velocity"])
            rhs.velocity = node["velocity"].as<std::vector<double>>();

        if (node["effort"])
            rhs.effort = node["effort"].as<std::vector<double>>();

        return true;
    }

    Node convert<sensor_msgs::MultiDOFJointState>::encode(const sensor_msgs::MultiDOFJointState &rhs)
    {
        Node node;
        node["header"] = rhs.header;

        node["joint_names"] = rhs.joint_names;
        node["joint_names"].SetStyle(YAML::EmitterStyle::Flow);

        node["transforms"] = rhs.transforms;
        node["transforms"].SetStyle(YAML::EmitterStyle::Flow);

        node["twist"] = rhs.twist;
        node["twist"].SetStyle(YAML::EmitterStyle::Flow);

        node["wrench"] = rhs.wrench;
        node["wrench"].SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    bool convert<sensor_msgs::MultiDOFJointState>::decode(const Node &node, sensor_msgs::MultiDOFJointState &rhs)
    {
        rhs = sensor_msgs::MultiDOFJointState();
        rhs.header = node["header"].as<std_msgs::Header>();

        if (node["joint_names"])
            rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

        if (node["transforms"])
            rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::Transform>>();

        if (node["twist"])
            rhs.twist = node["twist"].as<std::vector<geometry_msgs::Twist>>();

        if (node["wrench"])
            rhs.wrench = node["wrench"].as<std::vector<geometry_msgs::Wrench>>();

        return true;
    }

    Node convert<moveit_msgs::AttachedCollisionObject>::encode(const moveit_msgs::AttachedCollisionObject &rhs)
    {
        Node node;
        return node;
    }

    bool convert<moveit_msgs::AttachedCollisionObject>::decode(const Node &node,
                                                               moveit_msgs::AttachedCollisionObject &rhs)
    {
        return true;
    }

    Node convert<trajectory_msgs::JointTrajectory>::encode(const trajectory_msgs::JointTrajectory &rhs)
    {
        Node node;
        return node;
    }

    bool convert<trajectory_msgs::JointTrajectory>::decode(const Node &node, trajectory_msgs::JointTrajectory &rhs)
    {
        return true;
    }

    Node convert<trajectory_msgs::JointTrajectoryPoint>::encode(const trajectory_msgs::JointTrajectoryPoint &rhs)
    {
        Node node;
        return node;
    }

    bool convert<trajectory_msgs::JointTrajectoryPoint>::decode(const Node &node,
                                                                trajectory_msgs::JointTrajectoryPoint &rhs)
    {
        return true;
    }

    Node convert<moveit_msgs::CollisionObject>::encode(const moveit_msgs::CollisionObject &rhs)
    {
        Node node;
        return node;
    }

    bool convert<moveit_msgs::CollisionObject>::decode(const Node &node, moveit_msgs::CollisionObject &rhs)
    {
        return true;
    }

    Node convert<object_recognition_msgs::ObjectType>::encode(const object_recognition_msgs::ObjectType &rhs)
    {
        Node node;
        return node;
    }

    bool convert<object_recognition_msgs::ObjectType>::decode(const Node &node,
                                                              object_recognition_msgs::ObjectType &rhs)
    {
        return true;
    }

    Node convert<moveit_msgs::LinkPadding>::encode(const moveit_msgs::LinkPadding &rhs)
    {
        Node node;
        node["link_name"] = rhs.link_name;
        node["padding"] = rhs.padding;
        return node;
    }

    bool convert<moveit_msgs::LinkPadding>::decode(const Node &node, moveit_msgs::LinkPadding &rhs)
    {
        rhs = moveit_msgs::LinkPadding();

        if (node["link_name"])
            rhs.link_name = node["link_name"].as<std::string>();

        if (node["padding"])
            rhs.padding = node["padding"].as<double>();

        return true;
    }

    Node convert<moveit_msgs::LinkScale>::encode(const moveit_msgs::LinkScale &rhs)
    {
        Node node;
        node["link_name"] = rhs.link_name;
        node["scale"] = rhs.scale;
        return node;
    }

    bool convert<moveit_msgs::LinkScale>::decode(const Node &node, moveit_msgs::LinkScale &rhs)
    {
        rhs = moveit_msgs::LinkScale();

        if (node["link_name"])
            rhs.link_name = node["link_name"].as<std::string>();

        if (node["scale"])
            rhs.scale = node["scale"].as<double>();

        return true;
    }

    Node convert<moveit_msgs::ObjectColor>::encode(const moveit_msgs::ObjectColor &rhs)
    {
        Node node;
        node["id"] = rhs.id;
        node["color"] = rhs.color;
        return node;
    }

    bool convert<moveit_msgs::ObjectColor>::decode(const Node &node, moveit_msgs::ObjectColor &rhs)
    {
        rhs = moveit_msgs::ObjectColor();

        if (node["id"])
            rhs.id = node["id"].as<std::string>();

        if (node["color"])
            rhs.color = node["color"].as<std_msgs::ColorRGBA>();

        return true;
    }

    Node convert<std_msgs::ColorRGBA>::encode(const std_msgs::ColorRGBA &rhs)
    {
        Node node;
        node.SetStyle(YAML::EmitterStyle::Flow);

        node.push_back(rhs.r);
        node.push_back(rhs.g);
        node.push_back(rhs.b);
        node.push_back(rhs.a);
        return node;
    }

    bool convert<std_msgs::ColorRGBA>::decode(const Node &node, std_msgs::ColorRGBA &rhs)
    {
        rhs = std_msgs::ColorRGBA();

        rhs.r = node[0].as<double>();
        rhs.g = node[1].as<double>();
        rhs.b = node[2].as<double>();
        rhs.a = node[3].as<double>();
        return true;
    }

    Node convert<moveit_msgs::AllowedCollisionMatrix>::encode(const moveit_msgs::AllowedCollisionMatrix &rhs)
    {
        Node node;
        return node;
    }

    bool convert<moveit_msgs::AllowedCollisionMatrix>::decode(const Node &node,
                                                              moveit_msgs::AllowedCollisionMatrix &rhs)
    {
        return true;
    }

    Node convert<moveit_msgs::PlanningSceneWorld>::encode(const moveit_msgs::PlanningSceneWorld &rhs)
    {
        Node node;
        return node;
    }

    bool convert<moveit_msgs::PlanningSceneWorld>::decode(const Node &node, moveit_msgs::PlanningSceneWorld &rhs)
    {
        return true;
    }
}  // namespace YAML
