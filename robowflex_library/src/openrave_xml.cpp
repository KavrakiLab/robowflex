
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/CollisionObject.h>

#include <sstream>
#include <tinyxml.h>

#include <robowflex/openrave_xml.h>
#include <ros/ros.h>

using namespace openrave;

std::vector<double> splitAndParse(std::string spacelist)
{
    std::stringstream ss(spacelist);
    std::string buf;

    std::vector<double> tokens;
    while (ss >> buf)
        tokens.push_back(std::stod(buf));

    return tokens;
}

Eigen::Affine3d TFfromXML(TiXmlElement *transElem, TiXmlElement *rotationElem, TiXmlElement *quatElem)
{
    Eigen::Affine3d tf = Eigen::Affine3d::Identity();
    if (transElem)
    {
        std::vector<double> trans = splitAndParse(std::string(transElem->GetText()));
        Eigen::Vector3d v(trans[0], trans[1], trans[2]);
        tf.translation() = v;
    }

    if (rotationElem)
    {
        std::vector<double> rot = splitAndParse(std::string(rotationElem->GetText()));

        Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
        axis.normalize();
        Eigen::Matrix3d angle_axis_matrix = Eigen::AngleAxisd(rot[3] * 3.1415926 / 180, axis).toRotationMatrix();

        tf.linear() = angle_axis_matrix;
    }

    if (quatElem)
    {
        std::vector<double> quat = splitAndParse(std::string(quatElem->GetText()));
        Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
        q.normalize();
        tf.linear() = q.toRotationMatrix();
    }

    return tf;
}

bool parse_kinbody(LoadIntoMoveIt& load_struct, TiXmlElement *elem, Eigen::Affine3d tf)
{
    if (not elem)
    {
        ROS_ERROR("Element doesn't exist?");
        return false;
    }
    TiXmlHandle hElem(elem);
    TiXmlElement *transElem = hElem.FirstChild("Translation").Element();
    TiXmlElement *rotElem = hElem.FirstChild("Rotation").Element();
    Eigen::Affine3d this_tf = TFfromXML(transElem, rotElem, nullptr);

    const char *filename = elem->Attribute("file");
    if (filename)
    {
        // We need to read in another file to get the actual info.
        // Hardcode the directory at the moment.
        std::string fullPath = std::string("/home/brycew/moveit_ws/src/OptPlanners_OpenRAVE/scripts/data/envs/") +
                               std::string(filename);
        TiXmlDocument doc(fullPath.c_str());
        if (!doc.LoadFile())
        {
            ROS_ERROR("Cannot load file %s", filename);
            return false;
        }
        TiXmlHandle hDoc(&doc);
        parse_kinbody(load_struct, hDoc.FirstChildElement("KinBody").Element(), tf * this_tf);

        return true;
    }

    TiXmlElement *bodyElem = hElem.FirstChild().Element();
    for (bodyElem; bodyElem; bodyElem = bodyElem->NextSiblingElement())
    {
        if (std::string(bodyElem->Value()) == "Body")
        {
            ROS_INFO("Pushing back collision object");
            moveit_msgs::CollisionObject coll_obj;
            coll_obj.id = bodyElem->Attribute("name");

            TiXmlHandle hBody(bodyElem);
            TiXmlElement *geom = hBody.FirstChild("Geom").Element();
            if (not geom)
            {
                ROS_ERROR("Malformed File: No Geom attribute?");
                return false;
            }


            TiXmlHandle hGeom(geom);
            // Set Offset
            Eigen::Affine3d offset = this_tf * tf * TFfromXML(hGeom.FirstChild("translation").Element(), nullptr, hGeom.FirstChild("quat").Element());
            geometry_msgs::Pose pose_msg;
            tf::poseEigenToMsg(offset, pose_msg);

            // Set type.
            const char *geom_type=  geom->Attribute("type");
            if (not geom_type)
            {
                ROS_ERROR("Malformed File: No type attribute in geom element");
                return false;
            }

            // Set Dimensions
            if (geom_type == "trimesh")
            {
                // Set resource
                // TODO

                //dimensions_ = Eigen::Vector3d(1.0, 1.0, 1.0);

                coll_obj.mesh_poses.push_back(pose_msg);
            }

            //Geometry::ShapeType::Type type = Geometry::ShapeType::toType(geom_type);

            if (geom_type == "box")
            {
                TiXmlElement *extents = hGeom.FirstChild("extents").Element();
                if (not extents)
                {
                    ROS_ERROR("Malformed File: No extents in a box geometry.");
                    return false;
                }
                std::vector<double> extent_vec = splitAndParse(extents->GetText());
                shapes::Shape *shape = new shapes::Box(extent_vec[0] * 2.0, extent_vec[1] * 2.0, extent_vec[2] * 2.0);
                shapes::ShapeMsg msg;
                shapes::constructMsgFromShape(shape, msg);
                coll_obj.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(msg));
                coll_obj.primitive_poses.push_back(pose_msg);
            }

            coll_obj.operation = moveit_msgs::CollisionObject::ADD;

            load_struct.coll_objects.push_back(coll_obj);
        }
    }
    return true;
}

bool openrave::fromXMLFile(moveit_msgs::PlanningScene planning_scene, const std::string &file)
{
    LoadIntoMoveIt load_struct;
    // Hardcoded offset on WAM (see wam7.kinbody.xml)
    Eigen::Affine3d tf;
    tf.translation() = Eigen::Vector3d(0.0, -0.14, -0.346);
    tf.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();
    load_struct.robot_offset = tf;
    TiXmlDocument doc(file.c_str());
    if (!doc.LoadFile())
    {
        ROS_ERROR("Cannot load file %s", file.c_str());
        return false;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement *pElem;
    TiXmlHandle hRoot(0);

    pElem = hDoc.FirstChildElement("Environment").FirstChildElement("Robot").Element();
    if (pElem)
    {
        // Keep track of where the robot is.
        TiXmlHandle hRobot(pElem);
        TiXmlElement *transElem = hRobot.FirstChild("Translation").Element();
        TiXmlElement *rotationElem = hRobot.FirstChild("RotationAxis").Element();
        load_struct.robot_offset = load_struct.robot_offset * TFfromXML(transElem, rotationElem, nullptr);
    }

    pElem = hDoc.FirstChildElement("Environment").FirstChildElement().Element();
    if (not pElem)
    {
        ROS_ERROR("There is no/an empty environment element in this openrave scene.");
        return false;
    }

    for (pElem; pElem; pElem=pElem->NextSiblingElement())
    {
        const std::string pKey = std::string(pElem->Value());
        if (pKey == "KinBody")
        {
            if (!parse_kinbody(load_struct, pElem, load_struct.robot_offset.inverse()))
            {
                return false;
            }
        }
        else
        {
            ROS_INFO("Ignoring elements of value %s", pKey.c_str());
        }
    }
    auto rob_trans = load_struct.robot_offset.translation();
    ROS_INFO("At the end, we found a rob translation of (%f, %f, %f), and we found %zu objects", 
             rob_trans[0], rob_trans[1], rob_trans[2], load_struct.coll_objects.size());

    for (auto it : load_struct.coll_objects)
    {
        planning_scene.is_diff = true;
        planning_scene.world.collision_objects.push_back(it);
    }
    return true;
}