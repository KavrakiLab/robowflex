/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_
#define ROBOWFLEX_IO_

#include <string>   // for std::string
#include <utility>  // for std::pair
#include <fstream>  // for std::ofstream

#include <boost/date_time.hpp>  // for date operations

#include <yaml-cpp/yaml.h>  // for YAML parsing

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Resolves `package://` URLs to their canonical form.
         *  The path does not need to exist, but the package does. Can be used to write new files in packages.
         *  \param[in] path Path to resolve.
         *  \return The canonical path, or "" on failure.
         */
        const std::string resolvePackage(const std::string &path);

        /** \brief Resolves `package://` URLs and relative file paths to their canonical form.
         *  \param[in] path Path to resolve.
         *  \return The canonical path, or "" on failure.
         */
        const std::string resolvePath(const std::string &path);

        /** \brief Loads an XML or .xacro file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
         */
        const std::string loadXMLToString(const std::string &path);

        /** \brief Loads a .xacro file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
         */
        const std::string loadXacroToString(const std::string &path);

        /** \brief Loads a file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist).
         */
        const std::string loadFileToString(const std::string &path);

        /** \brief Runs a command \a cmd and returns stdout as a string.
         *  \param[in] cmd Command to run.
         *  \return Contents of stdout from \a cmd, or "" on failure.
         */
        const std::string runCommand(const std::string &cmd);

        /** \brief Loads a file to a YAML node.
         *  \param[in] path File to load.
         *  \return A pair, where the first is true on success false on failure, and second is the YAML node.
         */
        const std::pair<bool, YAML::Node> loadFileToYAML(const std::string &path);

        /** \brief Creates a file and opens an output stream. Creates directories if they do not exist.
         *  \param[out] out Output stream to initialize.
         *  \param[in] file File to create and open.
         */
        void createFile(std::ofstream &out, const std::string &file);

        /** \brief Get the hostname of the system.
         *  \return String of the hostname.
         */
        const std::string getHostname();

        /** \brief Get the current time (up to milliseconds)
         *  \return The time.
         */
        boost::posix_time::ptime getDate();

        /** \brief Write the contents of a YAML node out to a potentially new file.
         *  \param[in] node Node to write.
         *  \param[in] file Filename to open.
         *  \return True on success, false otherwise.
         */
        bool YAMLtoFile(const YAML::Node &node, const std::string &file);

        /** \brief Dump a message (or YAML convertable object) to a file.
         *  \param[in] msg Message to dump.
         *  \param[in] file File to dump message to.
         *  \tparam T Type of the message.
         */
        template <typename T>
        bool messageToYAMLFile(T &msg, const std::string &file)
        {
            YAML::Node yaml;
            yaml = msg;

            return YAMLtoFile(yaml, file);
        }

        /** \brief Load a message (or YAML convertable object) from a file.
         *  \param[out] msg Message to load into.
         *  \param[in] file File to load message from.
         *  \tparam T Type of the message.
         */
        template <typename T>
        bool YAMLFileToMessage(T &msg, const std::string &file)
        {
            const auto &result = IO::loadFileToYAML(file);

            if (!result.first)
                return false;

            msg = result.second.as<T>();
            return true;
        }
    }  // namespace IO
}  // namespace robowflex

#endif
