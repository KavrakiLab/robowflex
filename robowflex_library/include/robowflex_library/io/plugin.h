/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_PLUGIN_
#define ROBOWFLEX_IO_PLUGIN_

#include <memory>
#include <typeinfo>

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
            /** \brief Get the singleton instance of PluginManager.
             *  \return The singleton PluginManager.
             */
            static PluginManager &getInstance()
            {
                static PluginManager instance;
                return instance;
            }

            // non-copyable
            PluginManager(PluginManager const &) = delete;
            void operator=(PluginManager const &) = delete;

            /** \brief Load a plugin named \a plugin of type T.
             *  \param[in] plugin Name of the plugin to load.
             *  \tparam T The type of the plugin to load.
             *  \return A shared pointer to the loaded plugin on success, nullptr on failure.
             */
            template <typename T>
            std::shared_ptr<T> load(const std::string &plugin)
            {
                auto loader = getLoader<T>();
                std::shared_ptr<T> loaded(loader->createUnmanagedInstance(plugin));
                return loaded;
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
             *  \tparam T The type of plugin loader to get.
             *  \return A plugin loader that loads plugins of type \a T.
             */
            template <typename T>
            LoaderPtr<T> getLoader()
            {
                const std::string &type = typeid(T).name();  // Get name of type using typeinfo

                LoaderPtr<T> loader;

                auto cached = loaders_.find(type);
                if (cached != loaders_.end())
                    loader = std::dynamic_pointer_cast<Loader<T>>(cached->second);
                else
                {
                    ROS_INFO("Creating Class Loader for type %s!", type.c_str());
                    loader.reset(new pluginlib::ClassLoader<T>("robowflex_library", type));
                    loaders_[type] = std::static_pointer_cast<BaseLoader>(loader);
                }

                return loader;
            }

            std::map<std::string, BaseLoaderPtr> loaders_;  ///< Cached loaders
        };
    }  // namespace IO
}  // namespace robowflex

#endif
