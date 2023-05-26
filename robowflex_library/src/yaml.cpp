/* Author: Zachary Kingston */

#include <cstdint>

#include <algorithm>
#include <string>

#include <boost/algorithm/hex.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/macros.h>
#include <robowflex_library/yaml.h>

using namespace robowflex;

namespace
{
    static std::string boolToString(bool b)
    {
        return b ? "true" : "false";
    }

    static bool nodeToBool(const YAML::Node &n)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s == "true";
    }

    static bool isHeaderEmpty(const std_msgs::Header &h)
    {
        return h.seq == 0 && h.stamp.isZero() && h.frame_id == "world";
    }

    static std_msgs::Header getDefaultHeader()
    {
        std_msgs::Header msg;
        msg.frame_id = "world";
        return msg;
    }

    static unsigned int nodeToCollisionObject(const YAML::Node &n)
    {
        try
        {
            std::string s = n.as<std::string>();
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);

            if (s == "move")
                return moveit_msgs::CollisionObject::MOVE;
            if (s == "remove")
                return moveit_msgs::CollisionObject::REMOVE;
            if (s == "append")
                return moveit_msgs::CollisionObject::APPEND;

            return moveit_msgs::CollisionObject::ADD;
        }
        catch (const YAML::BadConversion &e)
        {
            // Sometimes it is specified as the int.
            int op = n.as<int>();
            switch (op)
            {
                case 0:
                    return moveit_msgs::CollisionObject::ADD;
                case 1:
                    return moveit_msgs::CollisionObject::REMOVE;
                case 2:
                    return moveit_msgs::CollisionObject::APPEND;
                case 3:
                    return moveit_msgs::CollisionObject::MOVE;
                default:
                    return moveit_msgs::CollisionObject::ADD;
            }
        }
    }

    static std::string primitiveTypeToString(const shape_msgs::SolidPrimitive &shape)
    {
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
            default:
                return "invalid";
                break;
        }
    }

    static void nodeToPrimitiveType(const YAML::Node &n, shape_msgs::SolidPrimitive &shape)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);

        if (s == "sphere")
            shape.type = shape_msgs::SolidPrimitive::SPHERE;
        else if (s == "cylinder")
            shape.type = shape_msgs::SolidPrimitive::CYLINDER;
        else if (s == "cone")
            shape.type = shape_msgs::SolidPrimitive::CONE;
        else if (s == "box")
            shape.type = shape_msgs::SolidPrimitive::BOX;
        else
            shape.type = n.as<int>();
    }

    static bool isVector3Zero(const geometry_msgs::Vector3 &v)
    {
        return v.x == 0 && v.y == 0 && v.z == 0;
    }

    static bool isConstraintEmpty(const moveit_msgs::Constraints &c)
    {
        return c.joint_constraints.empty()           //
               && c.position_constraints.empty()     //
               && c.orientation_constraints.empty()  //
               && c.visibility_constraints.empty();
    }

    static std::string compressHex(const std::vector<int8_t> &v)
    {
        std::vector<char> compress;
        {
            boost::iostreams::filtering_ostream fos;
            fos.push(boost::iostreams::zlib_compressor());
            fos.push(boost::iostreams::back_inserter(compress));

            for (const auto &i : v)
                fos << i;
        }

        std::string result;
        boost::algorithm::hex(compress.begin(), compress.end(), std::back_inserter(result));

        return result;
    }

    static std::vector<int8_t> decompressHex(const std::string &hex)
    {
        std::vector<int8_t> unhexed;
        boost::algorithm::unhex(hex, std::back_inserter(unhexed));

        std::vector<char> decompress;
        {
            boost::iostreams::filtering_ostream fos;
            fos.push(boost::iostreams::zlib_decompressor());
            fos.push(boost::iostreams::back_inserter(decompress));

            for (const auto &i : unhexed)
                fos << i;
        }

        return std::vector<int8_t>(decompress.begin(), decompress.end());
    }
}  // namespace

namespace YAML
{
#if ROBOWFLEX_AT_LEAST_KINETIC
#else
    // Copied from YAML_DEFINE_CONVERT_STREAMABLE
    Node convert<signed char>::encode(const signed char &rhs)
    {
        std::stringstream stream;
        stream.precision(std::numeric_limits<signed char>::digits10 + 1);
        stream << rhs;
        return Node(stream.str());
    }

    bool convert<signed char>::decode(const Node &node, signed char &rhs)
    {
        if (node.Type() != NodeType::Scalar)
            return false;

        const std::string &input = node.Scalar();
        std::stringstream stream(input);
        stream.unsetf(std::ios::dec);
        if ((stream >> rhs) && (stream >> std::ws).eof())
            return true;
        if (std::numeric_limits<signed char>::has_infinity)
        {
            if (conversion::IsInfinity(input))
            {
                rhs = std::numeric_limits<signed char>::infinity();
                return true;
            }
            else if (conversion::IsNegativeInfinity(input))
            {
                rhs = -std::numeric_limits<signed char>::infinity();
                return true;
            }
        }

        if (std::numeric_limits<signed char>::has_quiet_NaN && conversion::IsNaN(input))
        {
            rhs = std::numeric_limits<signed char>::quiet_NaN();
            return true;
        }

        return false;
    }
#endif

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

        if (IO::isNode(node["name"]))
            rhs.name = node["name"].as<std::string>();

        if (IO::isNode(node["robot_state"]))
            rhs.robot_state = node["robot_state"].as<moveit_msgs::RobotState>();

        if (IO::isNode(node["robot_model_name"]))
            rhs.robot_model_name = node["robot_model_name"].as<std::string>();

        if (IO::isNode(node["fixed_frame_transforms"]))
            rhs.fixed_frame_transforms =
                node["fixed_frame_transforms"].as<std::vector<geometry_msgs::TransformStamped>>();

        if (IO::isNode(node["allowed_collision_matrix"]))
            rhs.allowed_collision_matrix =
                node["allowed_collision_matrix"].as<moveit_msgs::AllowedCollisionMatrix>();

        if (IO::isNode(node["link_padding"]))
            rhs.link_padding = node["link_padding"].as<std::vector<moveit_msgs::LinkPadding>>();

        if (IO::isNode(node["link_scale"]))
            rhs.link_scale = node["link_scale"].as<std::vector<moveit_msgs::LinkScale>>();

        if (IO::isNode(node["object_colors"]))
            rhs.object_colors = node["object_colors"].as<std::vector<moveit_msgs::ObjectColor>>();

        if (IO::isNode(node["world"]))
            rhs.world = node["world"].as<moveit_msgs::PlanningSceneWorld>();

        if (IO::isNode(node["is_diff"]))
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

        if (IO::isNode(node["joint_state"]))
            rhs.joint_state = node["joint_state"].as<sensor_msgs::JointState>();

        if (IO::isNode(node["multi_dof_joint_state"]))
            rhs.multi_dof_joint_state = node["multi_dof_joint_state"].as<sensor_msgs::MultiDOFJointState>();

        if (IO::isNode(node["attached_collision_objects"]))
            rhs.attached_collision_objects =
                node["attached_collision_objects"].as<std::vector<moveit_msgs::AttachedCollisionObject>>();

        if (IO::isNode(node["is_diff"]))
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

    bool convert<geometry_msgs::TransformStamped>::decode(const Node &node,
                                                          geometry_msgs::TransformStamped &rhs)
    {
        rhs = geometry_msgs::TransformStamped();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["child_frame_id"]))
            rhs.child_frame_id = node["child_frame_id"].as<std::string>();

        if (IO::isNode(node["transform"]))
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
            node["stamp"]["secs"] = rhs.stamp.sec;
            node["stamp"]["nsecs"] = rhs.stamp.nsec;
        }

        if (rhs.frame_id != "world" && rhs.frame_id != "/world")
            node["frame_id"] = rhs.frame_id;

        return node;
    }

    bool convert<std_msgs::Header>::decode(const Node &node, std_msgs::Header &rhs)
    {
        rhs = std_msgs::Header();
        rhs.frame_id = "world";

        if (IO::isNode(node["seq"]))
            rhs.seq = node["seq"].as<int>();

        if (IO::isNode(node["stamp"]))
        {
            try
            {
                rhs.stamp.sec = node["stamp"]["sec"].as<int>();
                rhs.stamp.nsec = node["stamp"]["nsec"].as<int>();
            }
            catch (YAML::InvalidNode &e)
            {
                rhs.stamp.sec = node["stamp"]["secs"].as<int>();
                rhs.stamp.nsec = node["stamp"]["nsecs"].as<int>();
            }
        }

        if (IO::isNode(node["frame_id"]))
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

        if (IO::isNode(node["position"]))
            rhs.position = node["position"].as<geometry_msgs::Point>();

        if (IO::isNode(node["orientation"]))
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

        if (IO::isNode(node["header"]))
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

        if (IO::isNode(node["translation"]))
            rhs.translation = node["translation"].as<geometry_msgs::Vector3>();

        if (IO::isNode(node["rotation"]))
            rhs.rotation = node["rotation"].as<geometry_msgs::Quaternion>();

        return true;
    }

    Node convert<geometry_msgs::Point>::encode(const geometry_msgs::Point &rhs)
    {
        Node node;
        ROBOWFLEX_YAML_FLOW(node);

        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        return node;
    }

    bool convert<geometry_msgs::Point>::decode(const Node &node, geometry_msgs::Point &rhs)
    {
        rhs = geometry_msgs::Point();

        if (node.IsSequence())
        {
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();
        }
        else
        {
            rhs.x = node["x"].as<double>();
            rhs.y = node["y"].as<double>();
            rhs.z = node["z"].as<double>();
        }
        return true;
    }

    Node convert<geometry_msgs::Vector3>::encode(const geometry_msgs::Vector3 &rhs)
    {
        Node node;
        ROBOWFLEX_YAML_FLOW(node);

        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        return node;
    }

    bool convert<geometry_msgs::Vector3>::decode(const Node &node, geometry_msgs::Vector3 &rhs)
    {
        rhs = geometry_msgs::Vector3();

        if (node.IsSequence())
        {
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();
        }
        else
        {
            rhs.x = node["x"].as<double>();
            rhs.y = node["y"].as<double>();
            rhs.z = node["z"].as<double>();
        }

        return true;
    }

    Node convert<geometry_msgs::Quaternion>::encode(const geometry_msgs::Quaternion &rhs)
    {
        Node node;
        ROBOWFLEX_YAML_FLOW(node);

        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        node.push_back(rhs.w);
        return node;
    }

    bool convert<geometry_msgs::Quaternion>::decode(const Node &node, geometry_msgs::Quaternion &rhs)
    {
        rhs = geometry_msgs::Quaternion();

        if (node.IsSequence())
        {
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();
            rhs.w = node[3].as<double>();
        }
        else
        {
            rhs.x = node["x"].as<double>();
            rhs.y = node["y"].as<double>();
            rhs.z = node["z"].as<double>();
            rhs.w = node["w"].as<double>();
        }
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
            ROBOWFLEX_YAML_FLOW(node["name"]);
        }

        if (!rhs.position.empty())
        {
            node["position"] = rhs.position;
            ROBOWFLEX_YAML_FLOW(node["position"]);
        }

        if (!rhs.velocity.empty())
        {
            node["velocity"] = rhs.velocity;
            ROBOWFLEX_YAML_FLOW(node["velocity"]);
        }

        if (!rhs.effort.empty())
        {
            node["effort"] = rhs.effort;
            ROBOWFLEX_YAML_FLOW(node["effort"]);
        }

        return node;
    }

    bool convert<sensor_msgs::JointState>::decode(const Node &node, sensor_msgs::JointState &rhs)
    {
        rhs = sensor_msgs::JointState();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["name"]))
            rhs.name = node["name"].as<std::vector<std::string>>();

        if (IO::isNode(node["position"]))
            rhs.position = node["position"].as<std::vector<double>>();

        if (IO::isNode(node["velocity"]))
            rhs.velocity = node["velocity"].as<std::vector<double>>();

        if (IO::isNode(node["effort"]))
            rhs.effort = node["effort"].as<std::vector<double>>();

        return true;
    }

    Node convert<sensor_msgs::MultiDOFJointState>::encode(const sensor_msgs::MultiDOFJointState &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["joint_names"] = rhs.joint_names;
        ROBOWFLEX_YAML_FLOW(node["joint_names"]);

        node["transforms"] = rhs.transforms;
        ROBOWFLEX_YAML_FLOW(node["transforms"]);

        node["twist"] = rhs.twist;
        ROBOWFLEX_YAML_FLOW(node["twist"]);

        node["wrench"] = rhs.wrench;
        ROBOWFLEX_YAML_FLOW(node["wrench"]);
        return node;
    }

    bool convert<sensor_msgs::MultiDOFJointState>::decode(const Node &node,
                                                          sensor_msgs::MultiDOFJointState &rhs)
    {
        rhs = sensor_msgs::MultiDOFJointState();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["joint_names"]))
            rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

        if (IO::isNode(node["transforms"]))
            rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::Transform>>();

        if (IO::isNode(node["twist"]))
            rhs.twist = node["twist"].as<std::vector<geometry_msgs::Twist>>();

        if (IO::isNode(node["wrench"]))
            rhs.wrench = node["wrench"].as<std::vector<geometry_msgs::Wrench>>();

        return true;
    }

    Node
    convert<moveit_msgs::AttachedCollisionObject>::encode(const moveit_msgs::AttachedCollisionObject &rhs)
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

        if (IO::isNode(node["link_name"]))
            rhs.link_name = node["link_name"].as<std::string>();

        if (IO::isNode(node["object"]))
            rhs.object = node["object"].as<moveit_msgs::CollisionObject>();

        if (IO::isNode(node["touch_links"]))
            rhs.touch_links = node["touch_links"].as<std::vector<std::string>>();

        if (IO::isNode(node["detach_posture"]))
            rhs.detach_posture = node["detach_posture"].as<trajectory_msgs::JointTrajectory>();

        if (IO::isNode(node["weight"]))
            rhs.weight = node["weight"].as<double>();

        return true;
    }

    Node convert<trajectory_msgs::JointTrajectory>::encode(const trajectory_msgs::JointTrajectory &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["joint_names"] = rhs.joint_names;
        ROBOWFLEX_YAML_FLOW(node["joint_names"]);
        node["points"] = rhs.points;
        return node;
    }

    bool convert<trajectory_msgs::JointTrajectory>::decode(const Node &node,
                                                           trajectory_msgs::JointTrajectory &rhs)
    {
        rhs = trajectory_msgs::JointTrajectory();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["joint_names"]))
            rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

        if (IO::isNode(node["points"]))
            rhs.points = node["points"].as<std::vector<trajectory_msgs::JointTrajectoryPoint>>();

        return true;
    }

    Node
    convert<trajectory_msgs::JointTrajectoryPoint>::encode(const trajectory_msgs::JointTrajectoryPoint &rhs)
    {
        Node node;

        if (!rhs.positions.empty())
        {
            node["positions"] = rhs.positions;
            ROBOWFLEX_YAML_FLOW(node["positions"]);
        }

        if (!rhs.velocities.empty())
        {
            node["velocities"] = rhs.velocities;
            ROBOWFLEX_YAML_FLOW(node["velocities"]);
        }

        if (!rhs.accelerations.empty())
        {
            node["accelerations"] = rhs.accelerations;
            ROBOWFLEX_YAML_FLOW(node["accelerations"]);
        }

        if (!rhs.effort.empty())
        {
            node["effort"] = rhs.effort;
            ROBOWFLEX_YAML_FLOW(node["effort"]);
        }

        node["time_from_start"] = rhs.time_from_start;

        return node;
    }

    bool convert<trajectory_msgs::JointTrajectoryPoint>::decode(const Node &node,
                                                                trajectory_msgs::JointTrajectoryPoint &rhs)
    {
        if (IO::isNode(node["positions"]))
            rhs.positions = node["positions"].as<std::vector<double>>();

        if (IO::isNode(node["velocities"]))
            rhs.velocities = node["velocities"].as<std::vector<double>>();

        if (IO::isNode(node["accelerations"]))
            rhs.accelerations = node["accelerations"].as<std::vector<double>>();

        if (IO::isNode(node["effort"]))
            rhs.effort = node["effort"].as<std::vector<double>>();

        if (IO::isNode(node["time_from_start"]))
            rhs.time_from_start = node["time_from_start"].as<ros::Duration>();

        return true;
    }

    Node convert<moveit_msgs::CollisionObject>::encode(const moveit_msgs::CollisionObject &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["id"] = rhs.id;

#if ROBOWFLEX_MOVEIT_VERSION >= ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 6)
        node["pose"] = rhs.pose;
#endif

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
        RobotPose pose = TF::identity();
        geometry_msgs::Pose pose_msg = TF::poseEigenToMsg(pose);
        rhs = moveit_msgs::CollisionObject();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["id"]))
            rhs.id = node["id"].as<std::string>();

        if (IO::isNode(node["pose"]))
        {
            pose_msg = node["pose"].as<geometry_msgs::Pose>();
            pose = TF::poseMsgToEigen(pose_msg);
        }

#if ROBOWFLEX_MOVEIT_VERSION >= ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 6)
        rhs.pose = pose_msg;
#endif

        if (IO::isNode(node["type"]))
            rhs.type = node["type"].as<object_recognition_msgs::ObjectType>();

        if (IO::isNode(node["primitives"]))
        {
            rhs.primitives = node["primitives"].as<std::vector<shape_msgs::SolidPrimitive>>();
            rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::Pose>>();

            // If loading a newer format, add pose to include offset
#if ROBOWFLEX_MOVEIT_VERSION < ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 6)
            for (auto &primitive_pose : rhs.primitive_poses)
                primitive_pose = TF::poseEigenToMsg(pose * TF::poseMsgToEigen(primitive_pose));
#endif
        }

        if (IO::isNode(node["meshes"]))
        {
            rhs.meshes = node["meshes"].as<std::vector<shape_msgs::Mesh>>();
            rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::Pose>>();

            // If loading a newer format, add pose to include offset
#if ROBOWFLEX_MOVEIT_VERSION < ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 6)
            for (auto &mesh_pose : rhs.mesh_poses)
                mesh_pose = TF::poseEigenToMsg(pose * TF::poseMsgToEigen(mesh_pose));
#endif
        }

        if (IO::isNode(node["planes"]))
        {
            rhs.planes = node["planes"].as<std::vector<shape_msgs::Plane>>();
            rhs.plane_poses = node["plane_poses"].as<std::vector<geometry_msgs::Pose>>();

            // If loading a newer format, add pose to include offset
#if ROBOWFLEX_MOVEIT_VERSION < ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 6)
            for (auto &plane_pose : rhs.plane_poses)
                plane_pose = TF::poseEigenToMsg(pose * TF::poseMsgToEigen(plane_pose));
#endif
        }

        if (IO::isNode(node["operation"]))
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

        if (IO::isNode(node["key"]))
            rhs.key = node["key"].as<std::string>();

        if (IO::isNode(node["db"]))
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

        if (IO::isNode(node["link_name"]))
            rhs.link_name = node["link_name"].as<std::string>();

        if (IO::isNode(node["padding"]))
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

        if (IO::isNode(node["link_name"]))
            rhs.link_name = node["link_name"].as<std::string>();

        if (IO::isNode(node["scale"]))
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

        if (IO::isNode(node["id"]))
            rhs.id = node["id"].as<std::string>();

        if (IO::isNode(node["color"]))
            rhs.color = node["color"].as<std_msgs::ColorRGBA>();

        return true;
    }

    Node convert<std_msgs::ColorRGBA>::encode(const std_msgs::ColorRGBA &rhs)
    {
        Node node;
        ROBOWFLEX_YAML_FLOW(node);

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
        ROBOWFLEX_YAML_FLOW(node["entry_names"]);

        node["entry_values"] = rhs.entry_values;

        if (!rhs.default_entry_values.empty())
        {
            node["default_entry_names"] = rhs.entry_names;
            ROBOWFLEX_YAML_FLOW(node["default_entry_names"]);

            std::vector<std::string> default_entry_values;
            for (const auto &b : rhs.default_entry_values)
                default_entry_values.emplace_back(boolToString(b));

            node["default_entry_values"] = default_entry_values;
            ROBOWFLEX_YAML_FLOW(node["default_entry_values"]);
        }

        return node;
    }

    bool convert<moveit_msgs::AllowedCollisionMatrix>::decode(const Node &node,
                                                              moveit_msgs::AllowedCollisionMatrix &rhs)
    {
        rhs = moveit_msgs::AllowedCollisionMatrix();

        if (IO::isNode(node["entry_names"]))
            rhs.entry_names = node["entry_names"].as<std::vector<std::string>>();

        if (IO::isNode(node["entry_values"]))
            rhs.entry_values = node["entry_values"].as<std::vector<moveit_msgs::AllowedCollisionEntry>>();

        if (IO::isNode(node["default_entry_names"]))
            rhs.default_entry_names = node["default_entry_names"].as<std::vector<std::string>>();

        if (IO::isNode(node["default_entry_values"]))
        {
            const auto &dev = node["default_entry_values"];
            for (const auto &b : dev)
                rhs.default_entry_values.push_back(nodeToBool(b));
        }

        return true;
    }

    Node convert<moveit_msgs::AllowedCollisionEntry>::encode(const moveit_msgs::AllowedCollisionEntry &rhs)
    {
        Node node;
        std::vector<std::string> enabled;
        for (const auto &b : rhs.enabled)
            enabled.emplace_back(boolToString(b));

        node = enabled;
        ROBOWFLEX_YAML_FLOW(node);
        return node;
    }

    bool convert<moveit_msgs::AllowedCollisionEntry>::decode(const Node &node,
                                                             moveit_msgs::AllowedCollisionEntry &rhs)
    {
        rhs = moveit_msgs::AllowedCollisionEntry();

        for (const auto &b : node)
            rhs.enabled.push_back(nodeToBool(b));

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

    bool convert<moveit_msgs::PlanningSceneWorld>::decode(const Node &node,
                                                          moveit_msgs::PlanningSceneWorld &rhs)
    {
        rhs = moveit_msgs::PlanningSceneWorld();

        if (IO::isNode(node["collision_objects"]))
            rhs.collision_objects = node["collision_objects"].as<std::vector<moveit_msgs::CollisionObject>>();

        if (IO::isNode(node["octomap"]))
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
        node["data"] = compressHex(rhs.data);

        return node;
    }

    bool convert<octomap_msgs::Octomap>::decode(const Node &node, octomap_msgs::Octomap &rhs)
    {
        rhs = octomap_msgs::Octomap();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["binary"]))
            rhs.binary = nodeToBool(node["binary"]);

        if (IO::isNode(node["id"]))
            rhs.id = node["id"].as<std::string>();

        if (IO::isNode(node["resolution"]))
            rhs.resolution = node["resolution"].as<double>();

        if (IO::isNode(node["data"]))
        {
            // Load old octomap formats / direct YAML output
            if (node["data"].IsSequence())
            {
                auto temp = node["data"].as<std::vector<int>>();
                rhs.data = std::vector<int8_t>(temp.begin(), temp.end());
            }
            else
            {
                auto temp = node["data"].as<std::string>();
                rhs.data = decompressHex(temp);
            }
        }

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

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        if (IO::isNode(node["origin"]))
            rhs.origin = node["origin"].as<geometry_msgs::Pose>();

        if (IO::isNode(node["octomap"]))
            rhs.octomap = node["octomap"].as<octomap_msgs::Octomap>();

        return true;
    }

    Node convert<ros::Time>::encode(const ros::Time &rhs)
    {
        Node node;
        node = rhs.toSec();
        return node;
    }

    bool convert<ros::Time>::decode(const Node &node, ros::Time &rhs)
    {
        rhs.fromSec(node.as<double>());
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
        if (node.Type() == NodeType::Scalar)
        {
            rhs.fromSec(node.as<double>());
            return true;
        }
        try
        {
            rhs.sec = node["sec"].as<int>();
            rhs.nsec = node["nsec"].as<int>();
        }
        catch (YAML::InvalidNode &e)
        {
            rhs.sec = node["secs"].as<int>();
            rhs.nsec = node["nsecs"].as<int>();
        }
        return true;
    }

    Node convert<shape_msgs::SolidPrimitive>::encode(const shape_msgs::SolidPrimitive &rhs)
    {
        Node node;
        node["type"] = primitiveTypeToString(rhs);
        node["dimensions"] = rhs.dimensions;
        ROBOWFLEX_YAML_FLOW(node["dimensions"]);
        return node;
    }

    bool convert<shape_msgs::SolidPrimitive>::decode(const Node &node, shape_msgs::SolidPrimitive &rhs)
    {
        rhs = shape_msgs::SolidPrimitive();
        if (IO::isNode(node["type"]))
            nodeToPrimitiveType(node["type"], rhs);

        if (IO::isNode(node["dimensions"]))
            rhs.dimensions = node["dimensions"].as<std::vector<double>>();

        return true;
    }

    Node convert<shape_msgs::Mesh>::encode(const shape_msgs::Mesh &rhs)
    {
        Node node;
        node["triangles"] = rhs.triangles;
        node["vertices"] = rhs.vertices;

        return node;
    }

    bool convert<shape_msgs::Mesh>::decode(const Node &node, shape_msgs::Mesh &rhs)
    {
        rhs = shape_msgs::Mesh();
        if (IO::isNode(node["resource"]))
        {
            std::string resource = node["resource"].as<std::string>();
            Eigen::Vector3d dimensions{1, 1, 1};

            if (IO::isNode(node["dimensions"]))
            {
                Eigen::Vector3d load(node["dimensions"].as<std::vector<double>>().data());
                dimensions = load;
            }

            Geometry mesh(Geometry::ShapeType::Type::MESH, dimensions, resource);
            rhs = mesh.getMeshMsg();
        }
        else
        {
            if (IO::isNode(node["triangles"]))
                rhs.triangles = node["triangles"].as<std::vector<shape_msgs::MeshTriangle>>();
            if (IO::isNode(node["vertices"]))
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
        ROBOWFLEX_YAML_FLOW(node);
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
        ROBOWFLEX_YAML_FLOW(node["coef"]);
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

    bool convert<moveit_msgs::WorkspaceParameters>::decode(const Node &node,
                                                           moveit_msgs::WorkspaceParameters &rhs)
    {
        rhs = moveit_msgs::WorkspaceParameters();

        if (IO::isNode(node["header"]))
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

        if (IO::isNode(node["name"]))
            rhs.name = node["name"].as<std::string>();

        if (IO::isNode(node["joint_constraints"]))
            rhs.joint_constraints = node["joint_constraints"].as<std::vector<moveit_msgs::JointConstraint>>();

        if (IO::isNode(node["position_constraints"]))
            rhs.position_constraints =
                node["position_constraints"].as<std::vector<moveit_msgs::PositionConstraint>>();

        if (IO::isNode(node["orientation_constraints"]))
            rhs.orientation_constraints =
                node["orientation_constraints"].as<std::vector<moveit_msgs::OrientationConstraint>>();

        if (IO::isNode(node["visibility_constraints"]))
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

        if (IO::isNode(node["tolerance_above"]))
            rhs.tolerance_above = node["tolerance_above"].as<double>();
        else
            rhs.tolerance_above = std::numeric_limits<double>::epsilon();

        if (IO::isNode(node["tolerance_below"]))
            rhs.tolerance_below = node["tolerance_below"].as<double>();
        else
            rhs.tolerance_below = std::numeric_limits<double>::epsilon();

        if (IO::isNode(node["weight"]))
            rhs.weight = node["weight"].as<double>();
        else
            rhs.weight = 1;

        return true;
    }

    Node convert<moveit_msgs::PositionConstraint>::encode(const moveit_msgs::PositionConstraint &rhs)
    {
        Node node;
        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["link_name"] = rhs.link_name;

        if (!isVector3Zero(rhs.target_point_offset))
            node["target_point_offset"] = rhs.target_point_offset;

        node["constraint_region"] = rhs.constraint_region;

        if (rhs.weight < 1)
            node["weight"] = rhs.weight;

        return node;
    }

    bool convert<moveit_msgs::PositionConstraint>::decode(const Node &node,
                                                          moveit_msgs::PositionConstraint &rhs)
    {
        rhs = moveit_msgs::PositionConstraint();
        rhs.weight = 1;

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        rhs.link_name = node["link_name"].as<std::string>();
        if (IO::isNode(node["target_point_offset"]))
            rhs.target_point_offset = node["target_point_offset"].as<geometry_msgs::Vector3>();

        rhs.constraint_region = node["constraint_region"].as<moveit_msgs::BoundingVolume>();
        if (IO::isNode(node["weight"]))
            rhs.weight = node["weight"].as<double>();
        else
            rhs.weight = 1;

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

    bool convert<moveit_msgs::OrientationConstraint>::decode(const Node &node,
                                                             moveit_msgs::OrientationConstraint &rhs)
    {
        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();
        else
            rhs.header = getDefaultHeader();

        rhs.orientation = node["orientation"].as<geometry_msgs::Quaternion>();
        rhs.link_name = node["link_name"].as<std::string>();

        rhs.absolute_x_axis_tolerance = node["absolute_x_axis_tolerance"].as<double>();
        rhs.absolute_y_axis_tolerance = node["absolute_y_axis_tolerance"].as<double>();
        rhs.absolute_z_axis_tolerance = node["absolute_z_axis_tolerance"].as<double>();

        if (IO::isNode(node["weight"]))
            rhs.weight = node["weight"].as<double>();
        else
            rhs.weight = 1;

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

    bool convert<moveit_msgs::VisibilityConstraint>::decode(const Node &node,
                                                            moveit_msgs::VisibilityConstraint &rhs)
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

        if (IO::isNode(node["primitives"]))
        {
            rhs.primitives = node["primitives"].as<std::vector<shape_msgs::SolidPrimitive>>();
            rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::Pose>>();
        }

        if (IO::isNode(node["meshes"]))
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

    bool convert<moveit_msgs::TrajectoryConstraints>::decode(const Node &node,
                                                             moveit_msgs::TrajectoryConstraints &rhs)
    {
        rhs.constraints = node["constraints"].as<std::vector<moveit_msgs::Constraints>>();
        return true;
    }

    Node convert<moveit_msgs::MotionPlanRequest>::encode(const moveit_msgs::MotionPlanRequest &rhs)
    {
        Node node;

        if (!(isHeaderEmpty(rhs.workspace_parameters.header) &&
              isVector3Zero(rhs.workspace_parameters.min_corner) &&
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

        if (rhs.max_velocity_scaling_factor < 1)
            node["max_velocity_scaling_factor"] = rhs.max_velocity_scaling_factor;

        if (rhs.max_acceleration_scaling_factor < 1)
            node["max_acceleration_scaling_factor"] = rhs.max_acceleration_scaling_factor;

        return node;
    }

    bool convert<moveit_msgs::MotionPlanRequest>::decode(const Node &node,
                                                         moveit_msgs::MotionPlanRequest &rhs)
    {
        rhs = moveit_msgs::MotionPlanRequest();

        if (IO::isNode(node["workspace_parameters"]))
            rhs.workspace_parameters = node["workspace_parameters"].as<moveit_msgs::WorkspaceParameters>();

        if (IO::isNode(node["start_state"]))
            rhs.start_state = node["start_state"].as<moveit_msgs::RobotState>();

        if (IO::isNode(node["goal_constraints"]))
            rhs.goal_constraints = node["goal_constraints"].as<std::vector<moveit_msgs::Constraints>>();

        if (IO::isNode(node["path_constraints"]))
            rhs.path_constraints = node["path_constraints"].as<moveit_msgs::Constraints>();

        if (IO::isNode(node["trajectory_constraints"]))
            rhs.trajectory_constraints =
                node["trajectory_constraints"].as<moveit_msgs::TrajectoryConstraints>();

        if (IO::isNode(node["planner_id"]))
            rhs.planner_id = node["planner_id"].as<std::string>();

        if (IO::isNode(node["group_name"]))
            rhs.group_name = node["group_name"].as<std::string>();

        if (IO::isNode(node["num_planning_attempts"]))
            rhs.num_planning_attempts = node["num_planning_attempts"].as<int>();
        else
            rhs.num_planning_attempts = 0;

        if (IO::isNode(node["allowed_planning_time"]))
            rhs.allowed_planning_time = node["allowed_planning_time"].as<double>();
        else
            rhs.allowed_planning_time = 0;

        if (IO::isNode(node["max_velocity_scaling_factor"]))
            rhs.max_velocity_scaling_factor = node["max_velocity_scaling_factor"].as<double>();
        else
            rhs.max_velocity_scaling_factor = 1;

        if (IO::isNode(node["max_acceleration_scaling_factor"]))
            rhs.max_acceleration_scaling_factor = node["max_acceleration_scaling_factor"].as<double>();
        else
            rhs.max_acceleration_scaling_factor = 1;

        return true;
    }

    Node convert<trajectory_msgs::MultiDOFJointTrajectory>::encode(
        const trajectory_msgs::MultiDOFJointTrajectory &rhs)
    {
        Node node;

        if (!isHeaderEmpty(rhs.header))
            node["header"] = rhs.header;

        node["joint_names"] = rhs.joint_names;
        node["points"] = rhs.points;

        return node;
    }

    bool convert<trajectory_msgs::MultiDOFJointTrajectory>::decode(
        const Node &node, trajectory_msgs::MultiDOFJointTrajectory &rhs)
    {
        rhs = trajectory_msgs::MultiDOFJointTrajectory();

        if (IO::isNode(node["header"]))
            rhs.header = node["header"].as<std_msgs::Header>();

        if (IO::isNode(node["joint_names"]))
            rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

        if (IO::isNode(node["points"]))
            rhs.points = node["points"].as<std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>>();

        return true;
    }

    Node convert<trajectory_msgs::MultiDOFJointTrajectoryPoint>::encode(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint &rhs)
    {
        Node node;

        if (!rhs.transforms.empty())
            node["transforms"] = rhs.transforms;

        if (!rhs.velocities.empty())
            node["velocities"] = rhs.velocities;

        if (!rhs.accelerations.empty())
            node["accelerations"] = rhs.accelerations;

        node["time_from_start"] = rhs.time_from_start;

        return node;
    }

    bool convert<trajectory_msgs::MultiDOFJointTrajectoryPoint>::decode(
        const Node &node, trajectory_msgs::MultiDOFJointTrajectoryPoint &rhs)
    {
        rhs = trajectory_msgs::MultiDOFJointTrajectoryPoint();

        if (IO::isNode(node["transforms"]))
            rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::Transform>>();

        if (IO::isNode(node["velocities"]))
            rhs.velocities = node["velocities"].as<std::vector<geometry_msgs::Twist>>();

        if (IO::isNode(node["accelerations"]))
            rhs.accelerations = node["accelerations"].as<std::vector<geometry_msgs::Twist>>();

        rhs.time_from_start = node["time_from_start"].as<ros::Duration>();
        return true;
    }

    Node convert<moveit_msgs::RobotTrajectory>::encode(const moveit_msgs::RobotTrajectory &rhs)
    {
        Node node;

        if (!rhs.joint_trajectory.points.empty())
            node["joint_trajectory"] = rhs.joint_trajectory;

        if (!rhs.multi_dof_joint_trajectory.points.empty())
            node["multi_dof_joint_trajectory"] = rhs.multi_dof_joint_trajectory;

        return node;
    }

    bool convert<moveit_msgs::RobotTrajectory>::decode(const Node &node, moveit_msgs::RobotTrajectory &rhs)
    {
        rhs = moveit_msgs::RobotTrajectory();

        if (IO::isNode(node["joint_trajectory"]))
            rhs.joint_trajectory = node["joint_trajectory"].as<trajectory_msgs::JointTrajectory>();

        if (IO::isNode(node["multi_dof_joint_trajectory"]))
            rhs.multi_dof_joint_trajectory =
                node["multi_dof_joint_trajectory"].as<trajectory_msgs::MultiDOFJointTrajectory>();

        return true;
    }
}  // namespace YAML

namespace robowflex
{
    namespace IO
    {
        bool isNode(const YAML::Node &node)
        {
            try
            {
                bool r = node.IsDefined() and not node.IsNull();
                if (r)
                    try
                    {
                        r = node.as<std::string>() != "~";
                    }
                    catch (std::exception &e)
                    {
                    }

                return r;
            }
            catch (YAML::InvalidNode &e)
            {
                return false;
            }
        }

        moveit_msgs::RobotState robotStateFromNode(const YAML::Node &node)
        {
            return node.as<moveit_msgs::RobotState>();
        }

        YAML::Node toNode(const geometry_msgs::Pose &msg)
        {
            YAML::Node node;
            node = msg;
            return node;
        }

        geometry_msgs::Pose poseFromNode(const YAML::Node &node)
        {
            return node.as<geometry_msgs::Pose>();
        }

        YAML::Node toNode(const moveit_msgs::PlanningScene &msg)
        {
            YAML::Node node;
            node = msg;
            return node;
        }

        YAML::Node toNode(const moveit_msgs::MotionPlanRequest &msg)
        {
            YAML::Node node;
            node = msg;
            return node;
        }

        YAML::Node toNode(const moveit_msgs::RobotTrajectory &msg)
        {
            YAML::Node node;
            node = msg;
            return node;
        }

        YAML::Node toNode(const moveit_msgs::RobotState &msg)
        {
            YAML::Node node;
            node = msg;
            return node;
        }

        bool fromYAMLFile(moveit_msgs::PlanningScene &msg, const std::string &file)
        {
            return IO::YAMLFileToMessage(msg, file);
        }

        bool fromYAMLFile(moveit_msgs::MotionPlanRequest &msg, const std::string &file)
        {
            return IO::YAMLFileToMessage(msg, file);
        }

        bool fromYAMLFile(moveit_msgs::RobotState &msg, const std::string &file)
        {
            return IO::YAMLFileToMessage(msg, file);
        }
    }  // namespace IO
}  // namespace robowflex
