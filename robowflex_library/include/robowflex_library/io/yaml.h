/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_YAML_
#define ROBOWFLEX_IO_YAML_

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <yaml-cpp/yaml.h>

namespace robowflex
{
    namespace IO
    {
        /** \brief Checks if a key exists within a YAML node.
         *  \param[in] node Node to check.
         *  \return True if the node exists and is not null.
         */
        bool isNode(const YAML::Node &node);

        /** \brief Converts a robot state YAML to a robot_state message.
         *  \param[in] node Node to convert.
         *  \return The converted message.
         */
        moveit_msgs::RobotState robotStateFromNode(const YAML::Node &node);

        /** \brief Converts a pose message to YAML.
         *  \param[in] msg Message to convert.
         *  \return The converted message.
         */
        YAML::Node toNode(const geometry_msgs::Pose &msg);

        /** \brief Converts a pose YAML to a goemetry message.
         *  \param[in] node Node to convert.
         *  \return The converted message.
         */
        geometry_msgs::Pose poseFromNode(const YAML::Node &node);

        /** \brief Converts a planning scene message to YAML.
         *  \param[in] msg Message to convert.
         *  \return The converted message.
         */
        YAML::Node toNode(const moveit_msgs::PlanningScene &msg);

        /** \brief Converts a motion plan request to YAML.
         *  \param[in] msg Message to convert.
         *  \return The converted message.
         */
        YAML::Node toNode(const moveit_msgs::MotionPlanRequest &msg);

        /** \brief Converts a motion plan to YAML.
         *  \param[in] msg Message to convert.
         *  \return The converted message.
         */
        YAML::Node toNode(const moveit_msgs::RobotTrajectory &msg);

        /** \brief Converts a robot state to YAML.
         *  \param[in] msg Message to convert.
         *  \return The converted message.
         */
        YAML::Node toNode(const moveit_msgs::RobotState &msg);

        /** \brief Loads a planning scene from a YAML file.
         *  \param[out] msg Message to load into.
         *  \param[in] file File to load.
         *  \return True on success, false on failure.
         */
        bool fromYAMLFile(moveit_msgs::PlanningScene &msg, const std::string &file);

        /** \brief Loads a motion planning request from a YAML file.
         *  \param[out] msg Message to load into.
         *  \param[in] file File to load.
         *  \return True on success, false on failure.
         */
        bool fromYAMLFile(moveit_msgs::MotionPlanRequest &msg, const std::string &file);

        /** \brief Loads a robot state from a YAML file.
         *  \param[out] msg Message to load into.
         *  \param[in] file File to load.
         *  \return True on success, false on failure.
         */
        bool fromYAMLFile(moveit_msgs::RobotState &msg, const std::string &file);
    }  // namespace IO
}  // namespace robowflex

#endif
