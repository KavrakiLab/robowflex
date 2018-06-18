#ifndef ROBOWFLEX_OPENRAVE_XML_
#define ROBOWFLEX_OPENRAVE_XML_

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
namespace openrave
{
    //std::vector<double> splitAndParse(std::string spacelist);

    //Eigen::Affine3d TFfromXML(TiXmlElement *transElem, TiXmlElement *rotationElem, TiXmlElement *quatElem);

    struct LoadIntoMoveIt {
        Eigen::Affine3d robot_offset;
        std::vector<moveit_msgs::CollisionObject> coll_objects;
    };

    //void parse_kinbody(LoadIntoMoveIt &load_struct, TiXmlElement *elem, Eigen::Affine3d tf);

    bool fromXMLFile(moveit_msgs::PlanningScene planning_scene, const std::string &file);
} // end namespace OpenRAVE


#endif