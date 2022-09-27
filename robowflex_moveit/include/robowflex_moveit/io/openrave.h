#ifndef ROBOWFLEX_OPENRAVE_
#define ROBOWFLEX_OPENRAVE_

#include <moveit_msgs/PlanningScene.h>

namespace robowflex
{
    namespace openrave
    {
        /** \brief Loads a planning_scene from an OpenRAVE Environment XML
         *  \param[out] planning_scene The output MoveIt message that will be filled with the planning scene
         *  contents.
         *  \param[in] file The path to the OpenRAVE environment XML.
         *  \param[in] model_dir The path to the models directory, which should contain files referenced by
         * the passed in file. In OpenRAVE, "the root directory for all models files is the folder openrave is
         * launched at."
         *  \return True on success, false on failure.
         */
        bool fromXMLFile(moveit_msgs::PlanningScene &planning_scene, const std::string &file,
                         const std::string &model_dir);
    }  // namespace openrave
}  // namespace robowflex

#endif
