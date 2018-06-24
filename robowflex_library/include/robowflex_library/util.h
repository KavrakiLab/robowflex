/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_UTIL_
#define ROBOWFLEX_UTIL_

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/date_time.hpp>

namespace robowflex
{
    /** \brief Exception that contains a message and an error code.
     */
    class Exception : public std::exception
    {
    public:
        /** \brief Constructor.
         *  \param[in] value Error code.
         *  \param[in] message Error message.
         */
        Exception(int value, const std::string &message) : value_(value), message_(message)
        {
        }

        /** \brief Get error code.
         */
        int getValue() const
        {
            return value_;
        }

        /** \brief Get error message.
         */
        const std::string &getMessage() const
        {
            return message_;
        }

        virtual const char *what() const throw()
        {
            return message_.c_str();
        }

    protected:
        const int value_;            ///< Error code.
        const std::string message_;  ///< Error message.
    };

    /** \brief Start-up ROS.
     *  If Boost version is greater than 1.64, `rosmaster` is started if it is not already running. A signal
     *  handler for SIGINT and SIGSEGV is installed to gracefully exit.
     *  \param[in] argc Argument count forwarded to ros::init
     *  \param[in] argv Arguments forwarded to ros::init
     *  \param[in] name Name of ROS node.
     */
    void startROS(int argc, char **argv, const std::string &name = "robowflex");

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

        /** \brief Creates a file and opens an output stream.
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

            YAML::Emitter out;
            out << yaml;

            std::ofstream fout(file);
            fout << out.c_str();
            fout.close();

            return true;
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

        /** \brief `rosbag` management class to ease message saving and loading.
         */
        class Bag
        {
        public:
            /** \brief File modes
             */
            enum Mode
            {
                READ,  ///< Read-only
                WRITE  ///< Write-only
            };

            /** \brief Constructor.
             *  \param[in] file File to open or create.
             *  \param[in] mode Mode to open file in.
             */
            Bag(const std::string &file, Mode mode = WRITE)
              : mode_(mode)
              , file_((mode_ == WRITE) ? file : IO::resolvePath(file))
              , bag_(file_, (mode_ == WRITE) ? rosbag::bagmode::Write : rosbag::bagmode::Read)
            {
            }

            /** \brief Destructor.
             *  Closes opened bag.
             */
            ~Bag()
            {
                bag_.close();
            }

            /** \brief Adds a message to the bag under \a topic.
             *  \param[in] topic Topic to save message under.
             *  \param[in] msg Message to write.
             *  \tparam T Type of message.
             */
            template <typename T>
            bool addMessage(const std::string &topic, T msg)
            {
                if (mode_ == WRITE)
                {
                    bag_.write(topic, ros::Time::now(), msg);
                    return true;
                }

                return false;
            }

            /** \brief Gets messages from an opened bag. Returns all messages of type \a T from a list of
             *  topics \a topics.
             *  \param[in] topics List of topics to load messages from.
             *  \tparam T type of messages to load from topics.
             */
            template <typename T>
            std::vector<T> getMessages(const std::vector<std::string> &topics)
            {
                std::vector<T> msgs;

                if (mode_ != READ)
                    return msgs;

                rosbag::View view(bag_, rosbag::TopicQuery(topics));
                for (auto &msg : view)
                {
                    typename T::ConstPtr ptr = msg.instantiate<T>();
                    if (ptr != nullptr)
                        msgs.emplace_back(*ptr);
                }
            }

        private:
            const Mode mode_;         ///< Mode to open file in.
            const std::string file_;  ///< File opened.
            rosbag::Bag bag_;         ///< `rosbag` opened.
        };

        /** \brief ROS parameter server handler to handle namespacing and automatic parameter deletion.
         */
        class Handler
        {
        public:
            /** \brief Constructor.
             *  \param[in] name Name for namespace.
             */
            Handler(const std::string &name);

            // non-copyable
            Handler(Handler const &) = delete;
            void operator=(Handler const &) = delete;

            /** \brief Copy constructor. Handles namespacing.
             *  \param[in] handler Parent handler.
             *  \param[in] name Additional namespace to add to parent handler.
             */
            Handler(const IO::Handler &handler, const std::string &name = "");

            /** \brief Destructor.
             *  Deletes all parameters created through this handler.
             */
            ~Handler();

            /** \brief Loads the contents of a YAML node to the parameter server under a \a prefix.
             *  \param[in] node YAML node to load.
             *  \param[in] prefix Prefix to put YAML node under.
             */
            void loadYAMLtoROS(const YAML::Node &node, const std::string &prefix = "");

            /** \brief Sets a parameter on the parameter server.
             *  \param[in] key Key to store parameter under.
             *  \param[in] value Value to store.
             *  \tparam T Type of the \a value.
             */
            template <typename T>
            void setParam(const std::string &key, const T &value)
            {
                nh_.setParam(key, value);
                params_.emplace_back(key);
            }

            /** \brief Checks if the parameter server has \a key.
             *  \param[in] key Key to check.
             *  \return True if \a key exists, false otherwise.
             */
            bool hasParam(const std::string &key)
            {
                return nh_.hasParam(key);
            }

            /** \brief Gets a parameter from the parameter server.
             *  \param[in] key Key of parameter.
             *  \param[out] value Value to store.
             *  \tparam T Type of the \a value.
             */
            template <typename T>
            bool getParam(const std::string &key, const T &value)
            {
                return nh_.getParam(key, value);
            }

            /** \brief Gets the node handle.
             *  \return The node handle.
             */
            const ros::NodeHandle &getHandle() const
            {
                return nh_;
            }

            /** \brief Gets the name of the handler.
             *  \return The name of the handler.
             */
            const std::string &getName() const
            {
                return name_;
            }

            /** \brief Gets the namespace of the handler.
             *  \return The namespace of the handler.
             */
            const std::string &getNamespace() const
            {
                return namespace_;
            }

        private:
            /** \brief Generates a UUID for the handler.
             *  \return String of UUID.
             */
            static const std::string generateUUID();
            static const std::string UUID;  ///< UUID of handler.

            const std::string name_;       ///< Name of handler.
            const std::string namespace_;  ///< Full namespace of handler.
            ros::NodeHandle nh_;           ///< ROS node handle.

            std::vector<std::string> params_;  ///< Set parameter keys.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
