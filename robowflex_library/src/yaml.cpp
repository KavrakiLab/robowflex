#include <algorithm>
#include <string>

#include <geometric_shapes/shape_operations.h>

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

    bool isHeaderEmpty(const std_msgs::Header &h)
    {
        return h.seq == 0 && h.stamp.isZero() && h.frame_id == "world";
    }

    std_msgs::Header getDefaultHeader()
    {
        std_msgs::Header msg;
        msg.frame_id = "world";
        return msg;
    }

    unsigned int nodeToCollisionObject(const YAML::Node &n)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);

        if (s == "move")
            return moveit_msgs::CollisionObject::MOVE;
        else if (s == "remove")
            return moveit_msgs::CollisionObject::REMOVE;
        else if (s == "append")
            return moveit_msgs::CollisionObject::APPEND;
        else
            return moveit_msgs::CollisionObject::ADD;
    }

    const std::string primitiveTypeToString(const shape_msgs::SolidPrimitive &shape)
    {
        // geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value;
        switch (shape.type)
        {
            case shape_msgs::SolidPrimitive::BOX:
                return "box";
                break;
            case shape_msgs::SolidPrimitive::SPHERE:
                return "sphere";
                break;
            case shape_msgs::SolidPrimitive::CYLINDER:
                return "cylinder";
                break;
            case shape_msgs::SolidPrimitive::CONE:
                return "cone";
                break;
        }
    }

    void nodeToPrimitiveType(const YAML::Node &n, shape_msgs::SolidPrimitive &shape)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);

        if (s == "sphere")
            shape.type = shape_msgs::SolidPrimitive::SPHERE;
        else if (s == "cylinder")
            shape.type = shape_msgs::SolidPrimitive::CYLINDER;
        else if (s == "cone")
            shape.type = shape_msgs::SolidPrimitive::CONE;
        else
            shape.type = shape_msgs::SolidPrimitive::BOX;
    }

    bool isVector3Zero(const geometry_msgs::Vector3 &v)
    {
        return v.x == 0 && v.y == 0 && v.z == 0;
    }

    bool isConstraintEmpty(const moveit_msgs::Constraints &c)
    {
        return c.joint_constraints.empty()           //
               && c.position_constraints.empty()     //
               && c.orientation_constraints.empty()  //
               && c.visibility_constraints.empty();
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

        if (!rhs.world.collision_objects.empty() || !rhs.world.octomap.octomap.data.empty())
            node["world"] = rhs.world;

        if (rhs.is_diff)
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

        if (!rhs.joint_state.name.empty())
            node["joint_state"] = rhs.joint_state;

        if (!rhs.multi_dof_joint_state.joint_names.empty())
            node["multi_dof_joint_state"] = rhs.multi_dof_joint_state;

        if (!rhs.attached_collision_objects.empty())
            node["attached_collision_objects"] = rhs.attached_collision_objects;

        if (rhs.is_diff)
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

        if (!isHeaderEmpty(rhs.header))
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
        else
            rhs.header = getDefaultHeader();

        if (node["child_frame_id"])
            rhs.child_frame_id = node["child_frame_id"].as<std::string>();

        if (node["transform"])
            rhs.transform = node["transform"].as<geometry_msgs::Transform>();

        return true;
    }

    Node convert<std_msgs::Header>::encode(const std_msgs::Header &rhs)
    {
        Node node;
        if (rhs.seq != 0)
            node["seq"] = rhs.seq;

        if (!rhs.stamp.isZero())
        {
            node["stamp"]["sec"] = rhs.stamp.sec;
            node["stamp"]["nsec"] = rhs.stamp.nsec;
        }

        if (rhs.frame_id != "world")
            node["frame_id"] = rhs.frame_id;

        return node;
    }

    bool convert<std_msgs::Header>::decode(const Node &node, std_msgs::Header &rhs)
    {
        rhs = std_msgs::Header();
        rhs.frame_id = "world";

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

    Node convert<geometry_msgs::Pose>::encode(const geometry_msgs::Pose &rhs)
    {
        Node node;
        node["position"] = rhs.position;
        node["orientation"] = rhs.orientation;
        return node;
    }

    bool convert<geometry_msgs::Pose>::decode(const Node &node, geometry_msgs::Pose &rhs)
    {
        rhs = geometry_msgs::Pose();

        if (node["position"])
            rhs.position = node["position"].as<geometry_msgs::Point>();

        if (node["orientation"])
            rhs.orientation = node["orientation"].as<geometry_msgs::Quaternion>();

        return true;
    }

    Node convert<geometry_msgs::PoseStamped>::encode(const geometry_msgs::PoseStamped &rhs)
    {
        Node node;
        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["pose"] = rhs.pose;

        return node;
    }

    bool convert<geometry_msgs::PoseStamped>::decode(const Node &node, geometry_msgs::PoseStamped &rhs)
    {
        rhs = geometry_msgs::PoseStamped();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        rhs.pose = node["pose"].as<geometry_msgs::Pose>();

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

    Node convert<geometry_msgs::Point>::encode(const geometry_msgs::Point &rhs)
    {
        Node node;
        node.SetStyle(YAML::EmitterStyle::Flow);

        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        return node;
    }

    bool convert<geometry_msgs::Point>::decode(const Node &node, geometry_msgs::Point &rhs)
    {
        rhs = geometry_msgs::Point();

        rhs.x = node[0].as<double>();
        rhs.y = node[1].as<double>();
        rhs.z = node[2].as<double>();
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

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        if (!rhs.name.empty())
        {
            node["name"] = rhs.name;
            node["name"].SetStyle(YAML::EmitterStyle::Flow);
        }

        if (!rhs.position.empty())
        {
            node["position"] = rhs.position;
            node["position"].SetStyle(YAML::EmitterStyle::Flow);
        }

        if (!rhs.velocity.empty())
        {
            node["velocity"] = rhs.velocity;
            node["velocity"].SetStyle(YAML::EmitterStyle::Flow);
        }

        if (!rhs.effort.empty())
        {
            node["effort"] = rhs.effort;
            node["effort"].SetStyle(YAML::EmitterStyle::Flow);
        }

        return node;
    }

    bool convert<sensor_msgs::JointState>::decode(const Node &node, sensor_msgs::JointState &rhs)
    {
        rhs = sensor_msgs::JointState();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

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

        if (!isHeaderEmpty(rhs.header))
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

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

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
        node["link_name"] = rhs.link_name;
        node["object"] = rhs.object;

        if (!rhs.touch_links.empty())
            node["touch_links"] = rhs.touch_links;

        if (!rhs.detach_posture.points.empty())
            node["detach_posture"] = rhs.detach_posture;

        node["weight"] = rhs.weight;
        return node;
    }

    bool convert<moveit_msgs::AttachedCollisionObject>::decode(const Node &node,
                                                               moveit_msgs::AttachedCollisionObject &rhs)
    {
        rhs = moveit_msgs::AttachedCollisionObject();

        if (node["link_name"])
            rhs.link_name = node["link_name"].as<std::string>();

        if (node["object"])
            rhs.object = node["object"].as<moveit_msgs::CollisionObject>();

        if (node["touch_links"])
            rhs.touch_links = node["touch_links"].as<std::vector<std::string>>();

        if (node["detach_posture"])
            rhs.detach_posture = node["detach_posture"].as<trajectory_msgs::JointTrajectory>();

        if (node["weight"])
            rhs.weight = node["weight"].as<double>();

        return true;
    }

    Node convert<trajectory_msgs::JointTrajectory>::encode(const trajectory_msgs::JointTrajectory &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["joint_names"] = rhs.joint_names;
        node["joint_names"].SetStyle(YAML::EmitterStyle::Flow);
        node["points"] = rhs.points;
        return node;
    }

    bool convert<trajectory_msgs::JointTrajectory>::decode(const Node &node, trajectory_msgs::JointTrajectory &rhs)
    {
        rhs = trajectory_msgs::JointTrajectory();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (node["joint_names"])
            rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

        if (node["points"])
            rhs.points = node["points"].as<std::vector<trajectory_msgs::JointTrajectoryPoint>>();

        return true;
    }

    Node convert<trajectory_msgs::JointTrajectoryPoint>::encode(const trajectory_msgs::JointTrajectoryPoint &rhs)
    {
        Node node;

        if (!rhs.positions.empty())
        {
            node["positions"] = rhs.positions;
            node["positions"].SetStyle(YAML::EmitterStyle::Flow);
        }

        if (!rhs.velocities.empty())
        {
            node["velocities"] = rhs.velocities;
            node["velocities"].SetStyle(YAML::EmitterStyle::Flow);
        }

        if (!rhs.accelerations.empty())
        {
            node["accelerations"] = rhs.accelerations;
            node["accelerations"].SetStyle(YAML::EmitterStyle::Flow);
        }

        if (!rhs.effort.empty())
        {
            node["effort"] = rhs.effort;
            node["effort"].SetStyle(YAML::EmitterStyle::Flow);
        }

        node["time_from_start"] = rhs.time_from_start;

        return node;
    }

    bool convert<trajectory_msgs::JointTrajectoryPoint>::decode(const Node &node,
                                                                trajectory_msgs::JointTrajectoryPoint &rhs)
    {
        if (node["positions"])
            rhs.positions = node["positions"].as<std::vector<double>>();

        if (node["velocities"])
            rhs.velocities = node["velocities"].as<std::vector<double>>();

        if (node["accelerations"])
            rhs.accelerations = node["accelerations"].as<std::vector<double>>();

        if (node["effort"])
            rhs.effort = node["effort"].as<std::vector<double>>();

        if (node["time_from_start"])
            rhs.time_from_start = node["time_from_start"].as<ros::Duration>();

        return true;
    }

    Node convert<moveit_msgs::CollisionObject>::encode(const moveit_msgs::CollisionObject &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["id"] = rhs.id;

        if (!rhs.type.key.empty())
            node["type"] = rhs.type;

        if (!rhs.primitives.empty())
        {
            node["primitives"] = rhs.primitives;
            node["primitive_poses"] = rhs.primitive_poses;
        }

        if (!rhs.meshes.empty())
        {
            node["meshes"] = rhs.meshes;
            node["mesh_poses"] = rhs.mesh_poses;
        }

        if (!rhs.planes.empty())
        {
            node["planes"] = rhs.planes;
            node["plane_poses"] = rhs.plane_poses;
        }

        std::string s;
        switch (rhs.operation)
        {
            case moveit_msgs::CollisionObject::REMOVE:
                s = "remove";
                break;
            case moveit_msgs::CollisionObject::APPEND:
                s = "append";
                break;
            case moveit_msgs::CollisionObject::MOVE:
                s = "move";
                break;
            default:
                return node;
        }

        node["operation"] = s;
        return node;
    }

    bool convert<moveit_msgs::CollisionObject>::decode(const Node &node, moveit_msgs::CollisionObject &rhs)
    {
        rhs = moveit_msgs::CollisionObject();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (node["id"])
            rhs.id = node["id"].as<std::string>();

        if (node["type"])
            rhs.type = node["type"].as<object_recognition_msgs::ObjectType>();

        if (node["primitives"])
        {
            rhs.primitives = node["primitives"].as<std::vector<shape_msgs::SolidPrimitive>>();
            rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::Pose>>();
        }

        if (node["meshes"])
        {
            rhs.meshes = node["meshes"].as<std::vector<shape_msgs::Mesh>>();
            rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::Pose>>();
        }

        if (node["planes"])
        {
            rhs.planes = node["planes"].as<std::vector<shape_msgs::Plane>>();
            rhs.plane_poses = node["plane_poses"].as<std::vector<geometry_msgs::Pose>>();
        }

        if (node["operation"])
            rhs.operation = nodeToCollisionObject(node["operation"]);

        return true;
    }

    Node convert<object_recognition_msgs::ObjectType>::encode(const object_recognition_msgs::ObjectType &rhs)
    {
        Node node;
        node["key"] = rhs.key;
        node["db"] = rhs.db;
        return node;
    }

    bool convert<object_recognition_msgs::ObjectType>::decode(const Node &node,
                                                              object_recognition_msgs::ObjectType &rhs)
    {
        rhs = object_recognition_msgs::ObjectType();

        if (node["key"])
            rhs.key = node["key"].as<std::string>();

        if (node["db"])
            rhs.db = node["db"].as<std::string>();

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
        node["entry_names"] = rhs.entry_names;
        node["entry_names"].SetStyle(YAML::EmitterStyle::Flow);

        node["entry_values"] = rhs.entry_values;

        if (!rhs.default_entry_values.empty())
        {
            node["default_entry_names"] = rhs.entry_names;
            node["default_entry_names"].SetStyle(YAML::EmitterStyle::Flow);

            std::vector<std::string> default_entry_values;
            for (auto &b : rhs.default_entry_values)
                default_entry_values.emplace_back(boolToString(b));

            node["default_entry_values"] = default_entry_values;
            node["default_entry_values"].SetStyle(YAML::EmitterStyle::Flow);
        }

        return node;
    }

    bool convert<moveit_msgs::AllowedCollisionMatrix>::decode(const Node &node,
                                                              moveit_msgs::AllowedCollisionMatrix &rhs)
    {
        rhs = moveit_msgs::AllowedCollisionMatrix();

        if (node["entry_names"])
            rhs.entry_names = node["entry_names"].as<std::vector<std::string>>();

        if (node["entry_values"])
            rhs.entry_values = node["entry_values"].as<std::vector<moveit_msgs::AllowedCollisionEntry>>();

        if (node["default_entry_names"])
            rhs.default_entry_names = node["default_entry_names"].as<std::vector<std::string>>();

        if (node["default_entry_values"])
        {
            const auto &dev = node["default_entry_values"];
            for (std::size_t i = 0; i < dev.size(); i++)
                rhs.default_entry_values.push_back(nodeToBool(dev[i]));
        }

        return true;
    }

    Node convert<moveit_msgs::AllowedCollisionEntry>::encode(const moveit_msgs::AllowedCollisionEntry &rhs)
    {
        Node node;
        std::vector<std::string> enabled;
        for (auto &b : rhs.enabled)
            enabled.emplace_back(boolToString(b));

        node = enabled;
        node.SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    bool convert<moveit_msgs::AllowedCollisionEntry>::decode(const Node &node, moveit_msgs::AllowedCollisionEntry &rhs)
    {
        rhs = moveit_msgs::AllowedCollisionEntry();

        for (std::size_t i = 0; i < node.size(); i++)
            rhs.enabled.push_back(nodeToBool(node[i]));

        return true;
    }

    Node convert<moveit_msgs::PlanningSceneWorld>::encode(const moveit_msgs::PlanningSceneWorld &rhs)
    {
        Node node;

        if (!rhs.collision_objects.empty())
            node["collision_objects"] = rhs.collision_objects;

        if (!rhs.octomap.octomap.data.empty())
            node["octomap"] = rhs.octomap;

        return node;
    }

    bool convert<moveit_msgs::PlanningSceneWorld>::decode(const Node &node, moveit_msgs::PlanningSceneWorld &rhs)
    {
        rhs = moveit_msgs::PlanningSceneWorld();

        if (node["collision_objects"])
            rhs.collision_objects = node["collision_objects"].as<std::vector<moveit_msgs::CollisionObject>>();

        if (node["octomap"])
            rhs.octomap = node["octomap"].as<octomap_msgs::OctomapWithPose>();

        return true;
    }

    Node convert<octomap_msgs::Octomap>::encode(const octomap_msgs::Octomap &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["binary"] = boolToString(rhs.binary);
        node["id"] = rhs.id;
        node["resolution"] = rhs.resolution;
        node["data"] = rhs.data;
        return node;
    }

    bool convert<octomap_msgs::Octomap>::decode(const Node &node, octomap_msgs::Octomap &rhs)
    {
        rhs = octomap_msgs::Octomap();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (node["binary"])
            rhs.binary = nodeToBool(node["binary"]);

        if (node["id"])
            rhs.id = node["id"].as<std::string>();

        if (node["resolution"])
            rhs.resolution = node["resolution"].as<double>();

        if (node["data"])
            rhs.data = node["data"].as<std::vector<signed char>>();

        return true;
    }

    Node convert<octomap_msgs::OctomapWithPose>::encode(const octomap_msgs::OctomapWithPose &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["origin"] = rhs.origin;
        node["octomap"] = rhs.octomap;
        return node;
    }

    bool convert<octomap_msgs::OctomapWithPose>::decode(const Node &node, octomap_msgs::OctomapWithPose &rhs)
    {
        rhs = octomap_msgs::OctomapWithPose();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (node["origin"])
            rhs.origin = node["origin"].as<geometry_msgs::Pose>();

        if (node["octomap"])
            rhs.octomap = node["octomap"].as<octomap_msgs::Octomap>();

        return true;
    }

    Node convert<ros::Duration>::encode(const ros::Duration &rhs)
    {
        Node node;
        node = rhs.toSec();
        return node;
    }

    bool convert<ros::Duration>::decode(const Node &node, ros::Duration &rhs)
    {
        rhs.fromSec(node.as<double>());
        return true;
    }

    Node convert<shape_msgs::SolidPrimitive>::encode(const shape_msgs::SolidPrimitive &rhs)
    {
        Node node;
        node["type"] = primitiveTypeToString(rhs);
        node["dimensions"] = rhs.dimensions;
        node["dimensions"].SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    bool convert<shape_msgs::SolidPrimitive>::decode(const Node &node, shape_msgs::SolidPrimitive &rhs)
    {
        rhs = shape_msgs::SolidPrimitive();
        if (node["type"])
            nodeToPrimitiveType(node["type"], rhs);

        if (node["dimensions"])
            rhs.dimensions = node["dimensions"].as<std::vector<double>>();

        return true;
    }

    Node convert<shape_msgs::Mesh>::encode(const shape_msgs::Mesh &rhs)
    {
        Node node;
        for (auto &triangle : rhs.triangles)
            node["triangles"].push_back(triangle);

        for (auto &vertex : rhs.vertices)
            node["vertices"].push_back(vertex);

        return node;
    }

    bool convert<shape_msgs::Mesh>::decode(const Node &node, shape_msgs::Mesh &rhs)
    {
        rhs = shape_msgs::Mesh();
        if (node["resource"])
        {
            std::string resource = node["resource"].as<std::string>();
            Eigen::Vector3d dimensions{1, 1, 1};

            if (node["dimensions"])
            {
                Eigen::Vector3d load(node["dimensions"].as<std::vector<double>>().data());
                dimensions = load;
            }

            shapes::Mesh *mesh = shapes::createMeshFromResource(resource, dimensions);

            shapes::ShapeMsg msg;
            shapes::constructMsgFromShape(mesh, msg);
            rhs = boost::get<shape_msgs::Mesh>(msg);
        }
        else
        {
            if (node["triangles"])
                rhs.triangles = node["triangles"].as<std::vector<shape_msgs::MeshTriangle>>();
            if (node["vertices"])
                rhs.vertices = node["vertices"].as<std::vector<geometry_msgs::Point>>();
        }
        return true;
    }

    Node convert<shape_msgs::MeshTriangle>::encode(const shape_msgs::MeshTriangle &rhs)
    {
        Node node;
        node.push_back(rhs.vertex_indices[0]);
        node.push_back(rhs.vertex_indices[1]);
        node.push_back(rhs.vertex_indices[2]);
        node.SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    bool convert<shape_msgs::MeshTriangle>::decode(const Node &node, shape_msgs::MeshTriangle &rhs)
    {
        rhs.vertex_indices[0] = node[0].as<double>();
        rhs.vertex_indices[1] = node[1].as<double>();
        rhs.vertex_indices[2] = node[2].as<double>();
        return true;
    }

    Node convert<shape_msgs::Plane>::encode(const shape_msgs::Plane &rhs)
    {
        Node node;
        node["coef"].push_back(rhs.coef[0]);
        node["coef"].push_back(rhs.coef[1]);
        node["coef"].push_back(rhs.coef[2]);
        node["coef"].push_back(rhs.coef[3]);
        node["coef"].SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    bool convert<shape_msgs::Plane>::decode(const Node &node, shape_msgs::Plane &rhs)
    {
        rhs.coef[0] = node["coef"][0].as<double>();
        rhs.coef[1] = node["coef"][1].as<double>();
        rhs.coef[2] = node["coef"][2].as<double>();
        rhs.coef[3] = node["coef"][3].as<double>();
        return true;
    }

    Node convert<moveit_msgs::WorkspaceParameters>::encode(const moveit_msgs::WorkspaceParameters &rhs)
    {
        Node node;
        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["min_corner"] = rhs.min_corner;
        node["max_corner"] = rhs.max_corner;
        return node;
    }

    bool convert<moveit_msgs::WorkspaceParameters>::decode(const Node &node, moveit_msgs::WorkspaceParameters &rhs)
    {
        rhs = moveit_msgs::WorkspaceParameters();

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        rhs.min_corner = node["min_corner"].as<geometry_msgs::Vector3>();
        rhs.max_corner = node["max_corner"].as<geometry_msgs::Vector3>();
        return true;
    }

    Node convert<moveit_msgs::Constraints>::encode(const moveit_msgs::Constraints &rhs)
    {
        Node node;

        if (!rhs.name.empty())
            node["name"] = rhs.name;

        if (!rhs.joint_constraints.empty())
            node["joint_constraints"] = rhs.joint_constraints;

        if (!rhs.position_constraints.empty())
            node["position_constraints"] = rhs.position_constraints;

        if (!rhs.orientation_constraints.empty())
            node["orientation_constraints"] = rhs.orientation_constraints;

        if (!rhs.visibility_constraints.empty())
            node["visibility_constraints"] = rhs.visibility_constraints;

        return node;
    }

    bool convert<moveit_msgs::Constraints>::decode(const Node &node, moveit_msgs::Constraints &rhs)
    {
        rhs = moveit_msgs::Constraints();

        if (node["name"])
            rhs.name = node["name"].as<std::string>();

        if (node["joint_constraints"])
            rhs.joint_constraints = node["joint_constraints"].as<std::vector<moveit_msgs::JointConstraint>>();

        if (node["position_constraints"])
            rhs.position_constraints = node["position_constraints"].as<std::vector<moveit_msgs::PositionConstraint>>();

        if (node["orientation_constraints"])
            rhs.orientation_constraints =
                node["orientation_constraints"].as<std::vector<moveit_msgs::OrientationConstraint>>();

        if (node["visibility_constraints"])
            rhs.visibility_constraints =
                node["visibility_constraints"].as<std::vector<moveit_msgs::VisibilityConstraint>>();

        return true;
    }

    Node convert<moveit_msgs::JointConstraint>::encode(const moveit_msgs::JointConstraint &rhs)
    {
        Node node;
        node["joint_name"] = rhs.joint_name;
        node["position"] = rhs.position;

        if (rhs.tolerance_above > std::numeric_limits<double>::epsilon())
            node["tolerance_above"] = rhs.tolerance_above;

        if (rhs.tolerance_below > std::numeric_limits<double>::epsilon())
            node["tolerance_below"] = rhs.tolerance_below;

        if (rhs.weight < 1)
            node["weight"] = rhs.weight;

        return node;
    }

    bool convert<moveit_msgs::JointConstraint>::decode(const Node &node, moveit_msgs::JointConstraint &rhs)
    {
        rhs.joint_name = node["joint_name"].as<std::string>();
        rhs.position = node["position"].as<double>();
        rhs.tolerance_above = node["tolerance_above"].as<double>();
        rhs.tolerance_below = node["tolerance_below"].as<double>();

        if (node["weight"])
            rhs.weight = node["weight"].as<double>();

        return true;
    }

    Node convert<moveit_msgs::PositionConstraint>::encode(const moveit_msgs::PositionConstraint &rhs)
    {
        Node node;
        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["link_name"] = rhs.link_name;

        if (isVector3Zero(rhs.target_point_offset))
            node["target_point_offset"] = rhs.target_point_offset;

        node["constraint_region"] = rhs.constraint_region;

        if (rhs.weight < 1)
            node["weight"] = rhs.weight;

        return node;
    }

    bool convert<moveit_msgs::PositionConstraint>::decode(const Node &node, moveit_msgs::PositionConstraint &rhs)
    {
        rhs = moveit_msgs::PositionConstraint();
        rhs.weight = 1;

        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        rhs.link_name = node["link_name"].as<std::string>();
        if (node["target_point_offset"])
            rhs.target_point_offset = node["target_point_offset"].as<geometry_msgs::Vector3>();

        rhs.constraint_region = node["constraint_region"].as<moveit_msgs::BoundingVolume>();
        if (node["weight"])
            rhs.weight = node["weight"].as<double>();

        return true;
    }

    Node convert<moveit_msgs::OrientationConstraint>::encode(const moveit_msgs::OrientationConstraint &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["orientation"] = rhs.orientation;
        node["link_name"] = rhs.link_name;

        node["absolute_x_axis_tolerance"] = rhs.absolute_x_axis_tolerance;
        node["absolute_y_axis_tolerance"] = rhs.absolute_y_axis_tolerance;
        node["absolute_z_axis_tolerance"] = rhs.absolute_z_axis_tolerance;

        if (rhs.weight < 1)
            node["weight"] = rhs.weight;

        return node;
    }

    bool convert<moveit_msgs::OrientationConstraint>::decode(const Node &node, moveit_msgs::OrientationConstraint &rhs)
    {
        if (node["header"])
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        rhs.orientation = node["orientation"].as<geometry_msgs::Quaternion>();
        rhs.link_name = node["link_name"].as<std::string>();

        rhs.absolute_x_axis_tolerance = node["absolute_x_axis_tolerance"].as<double>();
        rhs.absolute_y_axis_tolerance = node["absolute_y_axis_tolerance"].as<double>();
        rhs.absolute_z_axis_tolerance = node["absolute_z_axis_tolerance"].as<double>();

        if (node["weight"])
            rhs.weight = node["weight"].as<double>();

        return true;
    }

    Node convert<moveit_msgs::VisibilityConstraint>::encode(const moveit_msgs::VisibilityConstraint &rhs)
    {
        Node node;
        node["target_radius"] = rhs.target_radius;
        node["target_pose"] = rhs.target_pose;
        node["cone_sides"] = rhs.cone_sides;
        node["sensor_pose"] = rhs.sensor_pose;
        node["max_view_angle"] = rhs.max_view_angle;
        node["max_range_angle"] = rhs.max_range_angle;
        node["sensor_view_direction"] = rhs.sensor_view_direction;
        node["weight"] = rhs.weight;
        return node;
    }

    bool convert<moveit_msgs::VisibilityConstraint>::decode(const Node &node, moveit_msgs::VisibilityConstraint &rhs)
    {
        rhs = moveit_msgs::VisibilityConstraint();

        rhs.target_radius = node["target_radius"].as<double>();
        rhs.target_pose = node["target_pose"].as<geometry_msgs::PoseStamped>();
        rhs.cone_sides = node["cone_sides"].as<int>();
        rhs.sensor_pose = node["sensor_pose"].as<geometry_msgs::PoseStamped>();
        rhs.max_view_angle = node["max_view_angle"].as<double>();
        rhs.max_range_angle = node["max_range_angle"].as<double>();
        rhs.sensor_view_direction = node["sensor_view_direction"].as<int>();
        rhs.weight = node["weight"].as<double>();

        return true;
    }

    Node convert<moveit_msgs::BoundingVolume>::encode(const moveit_msgs::BoundingVolume &rhs)
    {
        Node node;

        if (!rhs.primitives.empty())
        {
            node["primitives"] = rhs.primitives;
            node["primitive_poses"] = rhs.primitive_poses;
        }

        if (!rhs.meshes.empty())
        {
            node["meshes"] = rhs.meshes;
            node["mesh_poses"] = rhs.mesh_poses;
        }

        return node;
    }

    bool convert<moveit_msgs::BoundingVolume>::decode(const Node &node, moveit_msgs::BoundingVolume &rhs)
    {
        rhs = moveit_msgs::BoundingVolume();

        if (node["primitives"])
        {
            rhs.primitives = node["primitives"].as<std::vector<shape_msgs::SolidPrimitive>>();
            rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::Pose>>();
        }

        if (node["meshes"])
        {
            rhs.meshes = node["meshes"].as<std::vector<shape_msgs::Mesh>>();
            rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::Pose>>();
        }

        return true;
    }

    Node convert<moveit_msgs::TrajectoryConstraints>::encode(const moveit_msgs::TrajectoryConstraints &rhs)
    {
        Node node;
        node["constraints"] = rhs.constraints;
        return node;
    }

    bool convert<moveit_msgs::TrajectoryConstraints>::decode(const Node &node, moveit_msgs::TrajectoryConstraints &rhs)
    {
        rhs.constraints = node["constraints"].as<std::vector<moveit_msgs::Constraints>>();
        return true;
    }

    Node convert<moveit_msgs::MotionPlanRequest>::encode(const moveit_msgs::MotionPlanRequest &rhs)
    {
        Node node;

        if (!(isHeaderEmpty(rhs.workspace_parameters.header) && isVector3Zero(rhs.workspace_parameters.min_corner) &&
              isVector3Zero(rhs.workspace_parameters.max_corner)))
            node["workspace_parameters"] = rhs.workspace_parameters;

        node["start_state"] = rhs.start_state;

        if (!rhs.goal_constraints.empty())
            node["goal_constraints"] = rhs.goal_constraints;

        if (!isConstraintEmpty(rhs.path_constraints))
            node["path_constraints"] = rhs.path_constraints;

        if (!rhs.trajectory_constraints.constraints.empty())
            node["trajectory_constraints"] = rhs.trajectory_constraints;

        if (!rhs.planner_id.empty())
            node["planner_id"] = rhs.planner_id;

        if (!rhs.group_name.empty())
            node["group_name"] = rhs.group_name;

        if (rhs.num_planning_attempts != 0)
            node["num_planning_attempts"] = rhs.num_planning_attempts;

        if (rhs.allowed_planning_time != 0)
            node["allowed_planning_time"] = rhs.allowed_planning_time;

        if (rhs.max_velocity_scaling_factor > 0)
            node["max_velocity_scaling_factor"] = rhs.max_velocity_scaling_factor;

        if (rhs.max_acceleration_scaling_factor > 0)
            node["max_acceleration_scaling_factor"] = rhs.max_acceleration_scaling_factor;

        return node;
    }

    bool convert<moveit_msgs::MotionPlanRequest>::decode(const Node &node, moveit_msgs::MotionPlanRequest &rhs)
    {
        rhs = moveit_msgs::MotionPlanRequest();

        if (node["workspace_parameters"])
            rhs.workspace_parameters = node["workspace_parameters"].as<moveit_msgs::WorkspaceParameters>();

        if (node["start_state"])
            rhs.start_state = node["start_state"].as<moveit_msgs::RobotState>();

        if (node["goal_constraints"])
            rhs.goal_constraints = node["goal_constraints"].as<std::vector<moveit_msgs::Constraints>>();

        if (node["path_constraints"])
            rhs.path_constraints = node["path_constraints"].as<moveit_msgs::Constraints>();

        if (node["trajectory_constraints"])
            rhs.trajectory_constraints = node["trajectory_constraints"].as<moveit_msgs::TrajectoryConstraints>();

        if (node["planner_id"])
            rhs.planner_id = node["planner_id"].as<std::string>();

        if (node["group_name"])
            rhs.group_name = node["group_name"].as<std::string>();

        if (node["num_planning_attempts"])
            rhs.num_planning_attempts = node["num_planning_attempts"].as<int>();

        if (node["allowed_planning_time"])
            rhs.allowed_planning_time = node["allowed_planning_time"].as<double>();

        if (node["max_velocity_scaling_factor"])
            rhs.max_velocity_scaling_factor = node["max_velocity_scaling_factor"].as<double>();

        if (node["max_acceleration_scaling_factor"])
            rhs.max_acceleration_scaling_factor = node["max_acceleration_scaling_factor"].as<double>();

        return true;
    }
}  // namespace YAML
