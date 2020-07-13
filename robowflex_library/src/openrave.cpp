/* Author: Bryce Willey */

#include <stack>
#include <algorithm>

#include <tinyxml2.h>

#include <boost/math/constants/constants.hpp>

#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/CollisionObject.h>

#include <robowflex_library/io.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/openrave.h>

using namespace robowflex;
using namespace robowflex::openrave;

namespace
{
    struct SceneParsingContext
    {
        RobotPose robot_offset;
        std::vector<moveit_msgs::CollisionObject> coll_objects;
        std::stack<std::string> directory_stack;
    };

    double toRadians(double v)
    {
        return v * boost::math::constants::pi<double>() / 180.;
    }

    template <typename T>
    tinyxml2::XMLElement *getFirstChild(T *elem, const std::string &name = "")
    {
        tinyxml2::XMLHandle handle(elem);
        tinyxml2::XMLElement *child;
        if (name.empty())
            child = handle.FirstChildElement().ToElement();
        else
            child = handle.FirstChildElement(name.c_str()).ToElement();

        return child;
    }

    RobotPose TFfromXML(tinyxml2::XMLElement *transElem, tinyxml2::XMLElement *rotationElem,
                        tinyxml2::XMLElement *quatElem)
    {
        RobotPose tf = RobotPose::Identity();
        if (transElem)
        {
            auto trans = IO::tokenize<double>(std::string(transElem->GetText()));
            tf.translation() = Eigen::Vector3d(trans[0], trans[1], trans[2]);
        }

        if (rotationElem)
        {
            auto rot = IO::tokenize<double>(std::string(rotationElem->GetText()));

            Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
            axis.normalize();

            tf.linear() = Eigen::AngleAxisd(toRadians(rot[3]), axis).toRotationMatrix();
        }

        if (quatElem)
        {
            auto quat = IO::tokenize<double>(std::string(quatElem->GetText()));

            Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
            q.normalize();

            tf.linear() = q.toRotationMatrix();
        }

        return tf;
    }

    bool parseKinbody(SceneParsingContext &load_struct, tinyxml2::XMLElement *elem, const RobotPose &tf,
                      moveit_msgs::PlanningScene &planning_scene)
    {
        if (not elem)
        {
            ROS_ERROR("Element doesn't exist?");
            return false;
        }

        auto transElem = getFirstChild(elem, "Translation");
        auto rotElem = getFirstChild(elem, "Rotation");
        RobotPose this_tf = TFfromXML(transElem, rotElem, nullptr);

        const char *filename = elem->Attribute("file");
        if (filename)
        {
            // We need to read in another file to get the actual info.
            std::string fullPath = load_struct.directory_stack.top() + "/" + std::string(filename);
            tinyxml2::XMLDocument doc;
            if (!doc.LoadFile(fullPath.c_str()))
            {
                ROS_ERROR("Cannot load file %s", fullPath.c_str());
                return false;
            }

            load_struct.directory_stack.push(IO::resolveParent(fullPath));
            return parseKinbody(load_struct, getFirstChild(&doc, "KinBody"), tf * this_tf, planning_scene);
        }

        tinyxml2::XMLElement *bodyElem = getFirstChild(elem);
        for (; bodyElem; bodyElem = bodyElem->NextSiblingElement())
        {
            if (std::string(bodyElem->Value()) == "Body")
            {
                ROS_INFO("Pushing back collision object");

                moveit_msgs::CollisionObject coll_obj;
                coll_obj.id = bodyElem->Attribute("name");
                coll_obj.header.frame_id = "world";

                tinyxml2::XMLElement *geom = getFirstChild(bodyElem, "Geom");
                if (not geom)
                {
                    ROS_ERROR("Malformed File: No Geom attribute?");
                    return false;
                }

                // Set Offset
                RobotPose offset =
                    this_tf * tf *
                    TFfromXML(getFirstChild(geom, "translation"), nullptr, getFirstChild(geom, "quat"));

                geometry_msgs::Pose pose_msg;
                tf::poseEigenToMsg(offset, pose_msg);

                // Set type.
                const char *geom_type = geom->Attribute("type");
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

                    tinyxml2::XMLElement *data = getFirstChild(geom, "Data");
                    std::string resourcePath;
                    if (data)
                        resourcePath = load_struct.directory_stack.top() + "/" + std::string(data->GetText());

                    else
                    {
                        tinyxml2::XMLElement *render = getFirstChild(geom, "Render");
                        if (render)
                            resourcePath =
                                load_struct.directory_stack.top() + "/" + std::string(data->GetText());
                        else
                        {
                            ROS_ERROR("Malformed File: No Data or Render Elements inside a trimesh Geom.");
                            return false;
                        }
                    }
                    auto mesh = Geometry::makeMesh(resourcePath, dimensions);

                    ROS_INFO("Setting mesh");
                    coll_obj.meshes.push_back(mesh->getMeshMsg());
                    coll_obj.mesh_poses.push_back(pose_msg);
                }

                ROS_INFO("Type: %s", geom_str.c_str());
                if (geom_str == "box")
                {
                    ROS_INFO("Setting box");
                    tinyxml2::XMLElement *extents_elem = getFirstChild(geom, "extents_elem");
                    if (not extents_elem)
                    {
                        ROS_ERROR("Malformed File: No extents_elem in a box geometry.");
                        return false;
                    }

                    auto extents = IO::tokenize<double>(extents_elem->GetText());
                    shapes::Shape *shape =
                        new shapes::Box(extents[0] * 2.0, extents[1] * 2.0, extents[2] * 2.0);
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
}  // namespace

bool openrave::fromXMLFile(moveit_msgs::PlanningScene &planning_scene, const std::string &file,
                           const std::string &model_dir)
{
    SceneParsingContext load_struct;
    load_struct.directory_stack.push(model_dir);

    // Hardcoded offset on WAM (see wam7.kinbody.xml)
    RobotPose tf;
    tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.346);
    tf.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();
    load_struct.robot_offset = tf;

    tinyxml2::XMLDocument doc;
    if (!doc.LoadFile(IO::resolvePath(file).c_str()))
    {
        ROS_ERROR("Cannot load file %s", file.c_str());
        return false;
    }

    auto env = getFirstChild(&doc, "Environment");
    auto robot = getFirstChild(env, "Robot");
    if (robot)
        load_struct.robot_offset =
            load_struct.robot_offset * TFfromXML(getFirstChild(robot, "Translation"),  //
                                                 getFirstChild(robot, "RotationAxis"), nullptr);

    auto elem = getFirstChild(env);
    if (not elem)
    {
        ROS_ERROR("There is no/an empty environment element in this openrave scene.");
        return false;
    }

    for (; elem; elem = elem->NextSiblingElement())
    {
        const std::string pKey = std::string(elem->Value());
        if (pKey == "KinBody")
        {
            if (!parseKinbody(load_struct, elem, load_struct.robot_offset.inverse(), planning_scene))
                return false;
        }
        else
            ROS_INFO("Ignoring elements of value %s", pKey.c_str());
    }

    auto rob_trans = load_struct.robot_offset.translation();
    ROS_INFO("At the end, we found a rob translation of (%f, %f, %f), and we found %zu objects", rob_trans[0],
             rob_trans[1], rob_trans[2], load_struct.coll_objects.size());

    if (not load_struct.coll_objects.empty())
        planning_scene.is_diff = true;

    return true;
}
