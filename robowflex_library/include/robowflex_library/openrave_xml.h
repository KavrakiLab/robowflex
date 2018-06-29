#ifndef ROBOWFLEX_OPENRAVE_XML_
#define ROBOWFLEX_OPENRAVE_XML_

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <stack>

namespace openrave
{

    //Eigen::Affine3d TFfromXML(TiXmlElement *transElem, TiXmlElement *rotationElem, TiXmlElement *quatElem);

    struct LoadIntoMoveIt {
        Eigen::Affine3d robot_offset;
        std::vector<moveit_msgs::CollisionObject> coll_objects;
        std::stack<std::string> directory_stack;
    };

    //void parse_kinbody(LoadIntoMoveIt &load_struct, TiXmlElement *elem, Eigen::Affine3d tf);

    /**
     * \brief Loads a planning_scene from an OpenRAVE Environment XML
     * \param planning_scene the output moveit message that will be filled with the planning scene contents.
     * \param file the path to the OpenRAVE env XML.
     * \param file the path to the models directory, which should contain files referenced by the passed in file.
     *        In OpenRAVE, "the root directory for all models files is the folder openrave is launched at." 
     */
    bool fromXMLFile(moveit_msgs::PlanningScene &planning_scene, const std::string &file, const std::string &model_dir);
} // end namespace OpenRAVE


#endif