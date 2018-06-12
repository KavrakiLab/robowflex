#include "robowflex.h"

using namespace robowflex;

namespace YAML
{
    template <>
    struct convert<moveit_msgs::PlanningScene>
    {
        static Node encode(const moveit_msgs::PlanningScene &rhs)
        {
            Node node;
            node["name"] = rhs.name;
            node["robot_state"] = rhs.robot_state;
            node["robot_model_name"] = rhs.robot_model_name;
            node["fixed_frame_transforms"] = rhs.fixed_frame_transforms;
            // node["allowed_collision_matrix"] = rhs.allowed_collision_matrix;
            // node["link_padding"] = rhs.link_padding;
            // node["link_scale"] = rhs.link_scale;
            // node["object_colors"] = rhs.object_colors;
            // node["world"] = rhs.world;
            node["is_diff"] = rhs.is_diff;
            return node;
        }

        static bool decode(const Node &node, moveit_msgs::PlanningScene &rhs)
        {
            rhs = moveit_msgs::PlanningScene();

            if (node["name"])
                rhs.name = node["name"].as<std::string>();

            if (node["robot_state"])
                rhs.robot_state = node["robot_state"].as<moveit_msgs::RobotState>();

            if (node["robot_model_name"])
                rhs.robot_model_name = node["robot_model_name"].as<std::string>();

            if (node["fixed_frame_transforms"])
            {
                const auto &fft = node["fixed_frame_transforms"];
                for (YAML::const_iterator it = fft.begin(); it != fft.end(); ++it)
                    rhs.fixed_frame_transforms.push_back(it->as<geometry_msgs::TransformStamped>());
            }

            // if (node["allowed_collision_matrix"])
            //     rhs.allowed_collision_matrix =
            //         node["allowed_collision_matrix"].as<moveit_msgs::AllowedCollisionMatrix>();

            // if (node["link_padding"])
            //     rhs.link_padding = node["link_padding"].as<std::vector<moveit_msgs::LinkPadding>>();

            // if (node["link_scale"])
            //     rhs.link_scale = node["link_scale"].as<std::vector<moveit_msgs::LinkScale>>();

            // if (node["object_colors"])
            //     rhs.object_colors = node["object_colors"].as<std::vector<moveit_msgs::ObjectColor>>();

            // if (node["world"])
            //     rhs.world = node["world"].as<std::vector<moveit_msgs::PlanningSceneWorld>>();

            if (node["is_diff"])
                rhs.is_diff = node["is_diff"].as<bool>();

            return true;
        }
    };

    template <>
    struct convert<moveit_msgs::RobotState>
    {
        static Node encode(const moveit_msgs::RobotState &rhs)
        {
            Node node;
            node["joint_state"] = rhs.joint_state;
            node["multi_dof_joint_state"] = rhs.multi_dof_joint_state;
            node["attached_collision_objects"] = rhs.attached_collision_objects;
            node["is_diff"] = rhs.is_diff;
            return node;
        }

        static bool decode(const Node &node, moveit_msgs::RobotState &rhs)
        {
            rhs = moveit_msgs::RobotState();

            if (node["joint_state"])
                rhs.joint_state = node["joint_state"].as<sensor_msgs::JointState>();

            if (node["multi_dof_joint_state"])
                rhs.multi_dof_joint_state = node["multi_dof_joint_state"].as<sensor_msgs::MultiDOFJointState>();

            if (node["attached_collision_objects"])
            {
                const auto &aco = node["attached_collision_objects"];
                for (YAML::const_iterator it = aco.begin(); it != aco.end(); ++it)
                    rhs.attached_collision_objects.push_back(it->as<moveit_msgs::AttachedCollisionObject>());
            }

            if (node["is_diff"])
                rhs.is_diff = node["is_diff"].as<bool>();

            return true;
        }
    };

    template <>
    struct convert<geometry_msgs::TransformStamped>
    {
        static Node encode(const geometry_msgs::TransformStamped &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, geometry_msgs::TransformStamped &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<std_msgs::Header>
    {
        static Node encode(const std_msgs::Header &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, std_msgs::Header &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<geometry_msgs::Transform>
    {
        static Node encode(const geometry_msgs::Transform &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, geometry_msgs::Transform &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<geometry_msgs::Vector3>
    {
        static Node encode(const geometry_msgs::Vector3 &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, geometry_msgs::Vector3 &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<geometry_msgs::Quaternion>
    {
        static Node encode(const geometry_msgs::Quaternion &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, geometry_msgs::Quaternion &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<geometry_msgs::Twist>
    {
        static Node encode(const geometry_msgs::Twist &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, geometry_msgs::Twist &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<geometry_msgs::Wrench>
    {
        static Node encode(const geometry_msgs::Wrench &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, geometry_msgs::Wrench &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<sensor_msgs::JointState>
    {
        static Node encode(const sensor_msgs::JointState &rhs)
        {
            Node node;
            node["name"] = rhs.name;
            node["position"] = rhs.position;
            node["velocity"] = rhs.velocity;
            node["effort"] = rhs.effort;
            return node;
        }

        static bool decode(const Node &node, sensor_msgs::JointState &rhs)
        {
            rhs = sensor_msgs::JointState();

            if (node["name"])
            {
                const auto &names = node["name"];
                for (YAML::const_iterator it = names.begin(); it != names.end(); ++it)
                    rhs.name.push_back(it->as<std::string>());
            }

            if (node["position"])
            {
                const auto &positions = node["position"];
                for (YAML::const_iterator it = positions.begin(); it != positions.end(); ++it)
                    rhs.position.push_back(it->as<double>());
            }

            if (node["velocity"])
            {
                const auto &velocities = node["velocity"];
                for (YAML::const_iterator it = velocities.begin(); it != velocities.end(); ++it)
                    rhs.velocity.push_back(it->as<double>());
            }

            if (node["effort"])
            {
                const auto &efforts = node["effort"];
                for (YAML::const_iterator it = efforts.begin(); it != efforts.end(); ++it)
                    rhs.effort.push_back(it->as<double>());
            }

            return true;
        }
    };

    template <>
    struct convert<sensor_msgs::MultiDOFJointState>
    {
        static Node encode(const sensor_msgs::MultiDOFJointState &rhs)
        {
            Node node;
            node["joint_names"] = rhs.joint_names;
            node["transforms"] = rhs.transforms;
            node["twist"] = rhs.twist;
            node["wrench"] = rhs.wrench;
            return node;
        }

        static bool decode(const Node &node, sensor_msgs::MultiDOFJointState &rhs)
        {
            rhs = sensor_msgs::MultiDOFJointState();

            if (node["joint_names"])
            {
                const auto &joint_names = node["joint_names"];
                for (YAML::const_iterator it = joint_names.begin(); it != joint_names.end(); ++it)
                    rhs.joint_names.push_back(it->as<std::string>());
            }

            if (node["transforms"])
            {
                const auto &transforms = node["transforms"];
                for (YAML::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
                    rhs.transforms.push_back(it->as<geometry_msgs::Transform>());
            }

            if (node["twist"])
            {
                const auto &twist = node["twist"];
                for (YAML::const_iterator it = twist.begin(); it != twist.end(); ++it)
                    rhs.twist.push_back(it->as<geometry_msgs::Twist>());
            }

            if (node["wrench"])
            {
                const auto &wrench = node["wrench"];
                for (YAML::const_iterator it = wrench.begin(); it != wrench.end(); ++it)
                    rhs.wrench.push_back(it->as<geometry_msgs::Wrench>());
            }

            return true;
        }
    };

    template <>
    struct convert<moveit_msgs::AttachedCollisionObject>
    {
        static Node encode(const moveit_msgs::AttachedCollisionObject &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, moveit_msgs::AttachedCollisionObject &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<trajectory_msgs::JointTrajectory>
    {
        static Node encode(const trajectory_msgs::JointTrajectory &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, trajectory_msgs::JointTrajectory &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<trajectory_msgs::JointTrajectoryPoint>
    {
        static Node encode(const trajectory_msgs::JointTrajectoryPoint &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, trajectory_msgs::JointTrajectoryPoint &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<moveit_msgs::CollisionObject>
    {
        static Node encode(const moveit_msgs::CollisionObject &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, moveit_msgs::CollisionObject &rhs)
        {
            return true;
        }
    };

    template <>
    struct convert<object_recognition_msgs::ObjectType>
    {
        static Node encode(const object_recognition_msgs::ObjectType &rhs)
        {
            Node node;
            return node;
        }

        static bool decode(const Node &node, object_recognition_msgs::ObjectType &rhs)
        {
            return true;
        }
    };
}  // namespace YAML
