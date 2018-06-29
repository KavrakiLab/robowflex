
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/CollisionObject.h>

#include <sstream>
#include <tinyxml.h>

#include <robowflex_library/robowflex.h>
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

bool parse_kinbody(LoadIntoMoveIt& load_struct, TiXmlElement *elem, Eigen::Affine3d tf, moveit_msgs::PlanningScene &planning_scene)
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
        std::string fullPath = load_struct.directory_stack.top() + "/" + std::string(filename);
        TiXmlDocument doc(fullPath.c_str());
        if (!doc.LoadFile())
        {
            ROS_ERROR("Cannot load file %s", fullPath.c_str());
            return false;
        }
        TiXmlHandle hDoc(&doc);
        load_struct.directory_stack.push(robowflex::IO::resolveParent(fullPath));
        return parse_kinbody(load_struct, hDoc.FirstChildElement("KinBody").Element(), tf * this_tf, planning_scene);
    }

    TiXmlElement *bodyElem = hElem.FirstChild().Element();
    for (bodyElem; bodyElem; bodyElem = bodyElem->NextSiblingElement())
    {
        if (std::string(bodyElem->Value()) == "Body")
        {
            ROS_INFO("Pushing back collision object");
            moveit_msgs::CollisionObject coll_obj;
            coll_obj.id = bodyElem->Attribute("name");
            coll_obj.header.frame_id = "world";

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

            std::string geom_str = std::string(geom_type);

            // Set Dimensions
            if (geom_str == "trimesh")
            {
                // Set resource
                Eigen::Vector3d dimensions{1, 1, 1};

                TiXmlElement *data = hGeom.FirstChild("Data").Element();
                std::string resourcePath;
                if (data)
                {
                    resourcePath = load_struct.directory_stack.top() + "/" + std::string(data->GetText());
                }
                else {
                    TiXmlElement *render = hGeom.FirstChild("Render").Element();
                    if (render)
                    {
                        resourcePath = load_struct.directory_stack.top() + "/" + std::string(data->GetText());
                    }
                    else
                    {
                        ROS_ERROR("Malformed File: No Data or Render Elements inside a trimesh Geom.");
                        return false;
                    }
                }
                robowflex::Geometry mesh(robowflex::Geometry::ShapeType::Type::MESH, dimensions, resourcePath);

                ROS_INFO("Setting mesh");
                coll_obj.meshes.push_back(mesh.getMeshMsg());
                coll_obj.mesh_poses.push_back(pose_msg);
            }

            ROS_INFO("Type: %s", geom_str.c_str());
            if (geom_str == "box")
            {
                ROS_INFO("Setting box");
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
            planning_scene.world.collision_objects.push_back(coll_obj);
        }
    }
    load_struct.directory_stack.pop();
    return true;
}

bool openrave::fromXMLFile(moveit_msgs::PlanningScene &planning_scene, const std::string &file, const std::string &model_dir)
{
    LoadIntoMoveIt load_struct;
    load_struct.directory_stack.push(model_dir);
    // Hardcoded offset on WAM (see wam7.kinbody.xml)
    Eigen::Affine3d tf;
    tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.346);
    tf.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();
    load_struct.robot_offset = tf;
    TiXmlDocument doc(robowflex::IO::resolvePath(file.c_str()));
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
            if (!parse_kinbody(load_struct, pElem, load_struct.robot_offset.inverse(), planning_scene))
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

    if (not load_struct.coll_objects.empty())
    {
        planning_scene.is_diff = true;
    }
    return true;
}