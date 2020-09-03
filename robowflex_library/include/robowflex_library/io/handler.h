/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_HANDLER_
#define ROBOWFLEX_IO_HANDLER_

#include <string>  // for std::string

#include <yaml-cpp/yaml.h>  // for YAML::Node

#include <ros/node_handle.h>  // for ros::NodeHandle

namespace robowflex
{
    namespace IO
    {
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
            bool hasParam(const std::string &key) const;

            /** \brief Gets a parameter from the parameter server.
             *  \param[in] key Key of parameter.
             *  \param[out] value Value to store.
             *  \tparam T Type of the \a value.
             */
            template <typename T>
            bool getParam(const std::string &key, T &value) const
            {
                return nh_.getParam(key, value);
            }

            /** \brief Gets the node handle.
             *  \return The node handle.
             */
            const ros::NodeHandle &getHandle() const;

            /** \brief Gets the name of the handler.
             *  \return The name of the handler.
             */
            const std::string &getName() const;

            /** \brief Gets the namespace of the handler.
             *  \return The namespace of the handler.
             */
            const std::string &getNamespace() const;

        private:
            static const std::string UUID;  ///< UUID of handler.

            const std::string name_;       ///< Name of handler.
            const std::string namespace_;  ///< Full namespace of handler.
            ros::NodeHandle nh_;           ///< ROS node handle.

            std::vector<std::string> params_;  ///< Set parameter keys.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
