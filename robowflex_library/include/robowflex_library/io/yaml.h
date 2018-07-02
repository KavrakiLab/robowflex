/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_YAML_
#define ROBOWFLEX_IO_YAML_

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <yaml-cpp/yaml.h>

/** This file isn't necessary, but used to cut down on compile times. */

namespace robowflex
{
    namespace IO
    {
        /** \brief Converts a pose message to YAML.
         *  \param[in] msg Message to convert.
         *  \return The converted message.
         */
        YAML::Node toNode(const geometry_msgs::Pose &msg);

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
    }  // namespace IO
}  // namespace robowflex

#endif
