#ifndef ROBOWFLEX_UTIL_
#define ROBOWFLEX_UTIL_

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/date_time.hpp>

namespace robowflex
{
    class Exception : public std::exception
    {
    public:
        Exception(int value, const std::string &message) : value_(value), message_(message)
        {
        }

        int getValue() const
        {
            return value_;
        }

        const std::string &getMessage() const
        {
            return message_;
        }

        virtual const char *what() const throw()
        {
            return message_.c_str();
        }

    protected:
        const int value_;
        const std::string message_;
    };

    void startROS(int argc, char **argv, const std::string &name = "robowflex");

    namespace IO
    {
        // Resolves `package://` URLs and returns canonical absolute path if path exists, otherwise ""
        const std::string resolvePath(const std::string &path);

        // Loads an XML (or xacro) file to a string. If path does not exist or bad format, ""
        const std::string loadXMLToString(const std::string &path);

        // Loads a xacro file to a string. If path does not exist or bad format, ""
        const std::string loadXacroToString(const std::string &path);

        // Loads a file to a string. If path does not exist or bad format, ""
        const std::string loadFileToString(const std::string &path);

        // Runs a command and grabs stdout to a string., If fail ""
        const std::string runCommand(const std::string &cmd);

        // Loads an YAML file to a YAML node. If path does not exist or bad format, false in first.
        const std::pair<bool, YAML::Node> loadFileToYAML(const std::string &path);

        // Creates a file
        void createFile(std::ofstream &out, const std::string &file);

        const std::string getHostname();

        boost::posix_time::ptime getDate();

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

        template <typename T>
        bool YAMLFileToMessage(T &msg, const std::string &file)
        {
            const auto &result = IO::loadFileToYAML(file);

            if (!result.first)
                return false;

            msg = result.second.as<T>();
            return true;
        }

        class Bag
        {
        public:
            enum Mode
            {
                READ,
                WRITE
            };

            Bag(const std::string &file, Mode mode = WRITE)
              : mode_(mode)
              , file_((mode_ == WRITE) ? file : IO::resolvePath(file))
              , bag_(file_, (mode_ == WRITE) ? rosbag::bagmode::Write : rosbag::bagmode::Read)
            {
            }

            ~Bag()
            {
                bag_.close();
            }

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
            const Mode mode_;
            const std::string file_;
            rosbag::Bag bag_;
        };

        class Handler
        {
        public:
            Handler(const std::string &name);

            Handler(Handler const &) = delete;
            void operator=(Handler const &) = delete;

            Handler(const IO::Handler &handler, const std::string &name);

            ~Handler();

            // Loads an YAML node to the ROS parameter server.
            void loadYAMLtoROS(const YAML::Node &node, const std::string &prefix = "");

            template <typename T>
            void setParam(const std::string &key, const T &value)
            {
                nh_.setParam(key, value);
                params_.emplace_back(key);
            }

            bool hasParam(const std::string &key)
            {
                return nh_.hasParam(key);
            }

            template <typename T>
            bool getParam(const std::string &key, const T &value)
            {
                return nh_.getParam(key, value);
            }

            const ros::NodeHandle &getHandle() const
            {
                return nh_;
            }

            const std::string &getName() const
            {
                return name_;
            }

            const std::string &getNamespace() const
            {
                return namespace_;
            }

            template <typename T>
            ros::Publisher advertise(const std::string &name)
            {
                return nh_.advertise<T>(name, 1000);
            }

        private:
            // Generates a UUID
            static const std::string generateUUID();
            static const std::string UUID;

            const std::string name_;
            const std::string namespace_;
            ros::NodeHandle nh_;

            std::vector<std::string> params_;
        };
    }  // namespace IO
}  // namespace robowflex

#endif
