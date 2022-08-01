/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_
#define ROBOWFLEX_IO_

#include <string>   // for std::string
#include <utility>  // for std::pair
#include <fstream>  // for std::ofstream

#include <boost/date_time.hpp>  // for date operations

#include <ros/message_traits.h>  // for message operations

#include <yaml-cpp/yaml.h>  // for YAML parsing

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Generates a UUID.
         *  \return String of UUID.
         */
        std::string generateUUID();

        /** \brief Resolves `package://` URLs to their canonical form.
         *  The path does not need to exist, but the package does. Can be used to write new files in packages.
         *  \param[in] path Path to resolve.
         *  \return The canonical path, or "" on failure.
         */
        std::string resolvePackage(const std::string &path);

        /** \brief Finds all package URIs within a string.
         *  \param[in] string String to search.
         *  \return List of package URIs.
         */
        std::set<std::string> findPackageURIs(const std::string &string);

        /** \brief Resolves `package://` URLs and relative file paths to their canonical form.
         *  \param[in] path Path to resolve.
         *  \return The canonical path, or "" on failure.
         */
        std::string resolvePath(const std::string &path);

        /** \brief Resolves `package://` URLs to get the directory this path is in.
         *  \param[in] path Path to get the parent of.
         *  \return The directory that this path is contained in, or "" on failure.
         */
        std::string resolveParent(const std::string &path);

        /** \brief Concatenates two elements of a path, a directory and a filename.
         *  \param[in] directory Path to use as directory. If there are other elements at the end of the
         *                       directory path, they will be removed.
         *  \param[in] filename Filename to add.
         *  \return The canonical path for this path.
         */
        std::string makeFilepath(const std::string &directory, const std::string &filename);

        /** \brief Loads an XML or .xacro file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
         */
        std::string loadXMLToString(const std::string &path);

        /** \brief Loads a .xacro file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
         */
        std::string loadXacroToString(const std::string &path);

        /** \brief Loads a file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist).
         */
        std::string loadFileToString(const std::string &path);

        /** \brief Runs a command \a cmd and returns stdout as a string.
         *  \param[in] cmd Command to run.
         *  \return Contents of stdout from \a cmd, or "" on failure.
         */
        std::string runCommand(const std::string &cmd);

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

        /** \brief Creates a file and opens an output stream. Creates directories if they do not exist.
         *  \param[out] out Output stream to initialize.
         *  \param[in] file File to create and open.
         */
        void createFile(std::ofstream &out, const std::string &file);

        /** \brief Creates a temporary file and opens an output stream.
         *  \param[out] out Output stream to initialize.
         *  \return Filename of temporary file.
         */
        std::string createTempFile(std::ofstream &out);

        /** \brief Deletes a file.
         *  \param[in] file File to delete.
         */
        void deleteFile(const std::string &file);

        /** \brief Lists of the contents of a directory.
         *  \param[in] directory Directory to list.
         *  \return A pair of a bool and a vector of strings of filenames of the directories contents. The
         * first element will be true on success, false on failure. These filenames are absolute paths.
         */
        std::pair<bool, std::vector<std::string>> listDirectory(const std::string &directory);

        /** \brief Get the hostname of the system.
         *  \return String of the hostname.
         */
        std::string getHostname();

        /** \brief Get the process ID of this process.
         *  \return The process ID.
         */
        std::size_t getProcessID();

        /** \brief Get the thread ID of the current thread.
         *  \return The thread ID.
         */
        std::size_t getThreadID();

        /** \brief Get the current time (up to milliseconds)
         *  \return The time.
         */
        boost::posix_time::ptime getDate();

        /** \brief Get a duration in seconds from two times.
         *  \param[in] start The start time.
         *  \param[in] finish The finish time.
         *  \return The time in seconds.
         */
        double getSeconds(boost::posix_time::ptime start, boost::posix_time::ptime finish);

        /** \brief Put the current thread to sleep for a desired amount of seconds.
         *  \param[in] seconds Seconds to sleep for.
         */
        void threadSleep(double seconds);

        /** \brief Separates a \a string into casted tokens, based upon \a separators.
         *  \tparam The type of element to cast strings into.
         *  \param[in] string String to tokenize.
         *  \param[in] separators Separators to split string on.
         *  \return The tokenized string.
         */
        template <typename T>
        std::vector<T> tokenize(const std::string &string, const std::string &separators = " ");

        /** \brief Write the contents of a YAML node out to a potentially new file.
         *  \param[in] node Node to write.
         *  \param[in] file Filename to open.
         *  \return True on success, false otherwise.
         */
        bool YAMLToFile(const YAML::Node &node, const std::string &file);

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

        /** \brief Compute MD5 hash of message.
         *  \param[in] msg Message to hash.
         *  \tparam T Type of the message.
         *  \return The hash of the message.
         */
        template <typename T>
        std::string getMessageMD5(T &msg)
        {
            return ros::message_traits::md5sum<T>(msg);
        }
    }  // namespace IO
}  // namespace robowflex

#endif
