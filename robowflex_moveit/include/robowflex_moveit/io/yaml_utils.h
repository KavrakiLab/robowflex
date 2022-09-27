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

        /** \brief Write the contents of a YAML node out to a potentially new file.
         *  \param[in] node Node to write.
         *  \param[in] file Filename to open.
         *  \return True on success, false otherwise.
         */
        bool YAMLToFile(const YAML::Node &node, const std::string &file);

        /** \brief Loads a file to a YAML node.
         *  \param[in] path File to load.
         *  \return A pair, where the first is true on success false on failure, and second is the YAML node.
         */
        std::pair<bool, YAML::Node> loadFileToYAML(const std::string &path);

        /** \brief Loads a file with multiple documents to a vector of YAML nodes.
         *  \param[in] path File to load.
         *  \return A pair, where the first is true on success false on failure, and second is the vector of
         * YAML nodes.
         */
        std::pair<bool, std::vector<YAML::Node>> loadAllFromFileToYAML(const std::string &path);

        /** \brief Compute hash of a YAML node.
         *  \param[in] node YAML to hash.
         *  \return A hashed value of the node. Note: not cryptographically secure.
         */
        std::size_t hashYAML(const YAML::Node &node);

        /** \brief Dump a message (or YAML convertable object) to a file.
         *  \param[in] msg Message to dump.
         *  \param[in] file File to dump message to.
         *  \tparam T Type of the message.
         *  \return True on success, false on failure.
         */
        template <typename T>
        bool messageToYAMLFile(T &msg, const std::string &file)
        {
            YAML::Node yaml;
            yaml = msg;

            return YAMLToFile(yaml, file);
        }

        /** \brief Load a message (or YAML convertable object) from a file.
         *  \param[out] msg Message to load into.
         *  \param[in] file File to load message from.
         *  \tparam T Type of the message.
         *  \return True on success, false on failure.
         */
        template <typename T>
        bool YAMLFileToMessage(T &msg, const std::string &file)
        {
            const auto &result = IO::loadFileToYAML(file);
            if (result.first)
                msg = result.second.as<T>();

            return result.first;
        }

        /** \brief Use YAML to compute a hash of message contents
         *  \param[in] msg Message to dump.
         *  \param[in] file File to dump message to.
         *  \tparam T Type of the message.
         *  \return True on success, false on failure.
         */
        template <typename T>
        std::size_t messageToYAMLHash(const T &msg)
        {
            YAML::Node yaml;
            yaml = msg;

            return hashYAML(yaml);
        }
    }  // namespace IO
}  // namespace robowflex

#endif
