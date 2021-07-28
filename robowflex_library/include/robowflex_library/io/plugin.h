/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_PLUGIN_
#define ROBOWFLEX_IO_PLUGIN_

#include <mutex>
#include <memory>
#include <typeinfo>

#include <ros/package.h>

#include <robowflex_library/log.h>
#include <robowflex_library/macros.h>

#include <pluginlib/class_loader.hpp>

namespace robowflex
{
    namespace IO
    {
        /** \brief A singleton class for dynamic loading classes through pluginlib.
         */
        class PluginManager
        {
        public:
            // non-copyable
            PluginManager(PluginManager const &) = delete;
            void operator=(PluginManager const &) = delete;

            /** \brief Get the singleton instance of PluginManager
             *  \return The singleton PluginManager.
             */
            static PluginManager &getInstance()
            {
                static PluginManager instance;
                return instance;
            }

            /** \brief Load a plugin named \a plugin of type T using the singleton instance.
             *  \param[in] package ROS package that exports plugin's base class \a T.
             *  \param[in] plugin Name of the plugin to load.
             *  \tparam T The type of the plugin to load.
             *  \return A shared pointer to the loaded plugin on success, nullptr on failure.
             */
            template <typename T>
            static std::shared_ptr<T> load(const std::string &package, const std::string &plugin)
            {
                return getInstance().loadPlugin<T>(package, plugin);
            }

            /** \brief Load a plugin named \a plugin of type T.
             *  \param[in] package ROS package that exports plugin's base class \a T.
             *  \param[in] plugin Name of the plugin to load.
             *  \tparam T The type of the plugin to load.
             *  \return A shared pointer to the loaded plugin on success, nullptr on failure.
             */
            template <typename T>
            std::shared_ptr<T> loadPlugin(const std::string &package, const std::string &plugin)
            {
                if (ros::package::getPath(package).empty())
                {
                    RBX_ERROR("Package `%s` does not exist.", package);
                    return nullptr;
                }

                auto loader = getLoader<T>(package);

                try
                {
                    std::shared_ptr<T> loaded(loader->createUnmanagedInstance(plugin));
                    return loaded;
                }
                catch (pluginlib::LibraryLoadException &e)
                {
                    RBX_ERROR("Failed to library: %s", e.what());
                    return nullptr;
                }
            }

        private:
            /** \brief A typed class loader.
             *  \tparam T The type of class to load via the plugin loader.
             */
            template <typename T>
            using Loader = pluginlib::ClassLoader<T>;

            /** \brief A shared pointer to a typed class loader.
             *  \tparam T The type of class to load via the plugin loader.
             */
            template <typename T>
            using LoaderPtr = std::shared_ptr<Loader<T>>;

            /** \brief The base class of the class loader.
             */
            using BaseLoader = pluginlib::ClassLoaderBase;

            /** \brief A shared pointer to the base class of the class loader.
             */
            using BaseLoaderPtr = std::shared_ptr<BaseLoader>;

            /** \brief Constructor
             */
            PluginManager()
            {
            }

            /** \brief Gets the plugin loader for a plugin type \a T.
             *  Grabs the loader from cached loaders if available, otherwise creates the plugin loader and
             * caches it.
             *  \param[in] package ROS package that exports class \a T.
             *  \tparam T The type of plugin loader to get.
             *  \return A plugin loader that loads plugins of type \a T.
             */
            template <typename T>
            LoaderPtr<T> getLoader(const std::string &package)
            {
                std::unique_lock<std::mutex> lock(mutex_);

                // Need to possibly demangle type name...
                const std::string &type = ROBOWFLEX_DEMANGLE(typeid(T).name());
                auto key = std::make_pair(package, type);

                LoaderPtr<T> loader;

                auto cached = loaders_.find(key);
                if (cached != loaders_.end())
                    loader = std::dynamic_pointer_cast<Loader<T>>(cached->second);
                else
                {
                    RBX_INFO("Creating Class Loader for type `%s` from package `%s`!", type, package);

                    loader.reset(new pluginlib::ClassLoader<T>(package, type));
                    loaders_[key] = std::static_pointer_cast<BaseLoader>(loader);
                }

                return loader;
            }

            std::mutex mutex_;                                                      ///< Class loading mutex
            std::map<std::pair<std::string, std::string>, BaseLoaderPtr> loaders_;  ///< Cached loaders
        };
    }  // namespace IO
}  // namespace robowflex

#endif
