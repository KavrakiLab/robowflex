#include "robowflex.h"

using namespace robowflex;

Scene::Scene(Robot &robot) : scene_(new planning_scene::PlanningScene(robot.getModel()))
{
}

Scene::Scene(const Scene &other) : scene_(other.getSceneConst())
{
}

void Scene::operator=(const Scene &other)
{
    scene_ = other.getSceneConst();
}

moveit_msgs::PlanningScene Scene::getMessage() const
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);
    return msg;
}

robot_state::RobotState &Scene::getCurrentState()
{
    return scene_->getCurrentStateNonConst();
}

collision_detection::AllowedCollisionMatrix &Scene::getACM()
{
    return scene_->getAllowedCollisionMatrixNonConst();
}

void Scene::updateCollisionObject(const std::string &name, const Geometry &geometry, const Eigen::Affine3d &pose)
{
    auto &world = scene_->getWorldNonConst();
    if (world->hasObject(name))
    {
        if (!world->moveShapeInObject(name, geometry.getShape(), pose))
            world->removeObject(name);
        else
            return;
    }

    world->addToObject(name, geometry.getShape(), pose);
}

void Scene::removeCollisionObject(const std::string &name)
{
    scene_->getWorldNonConst()->removeObject(name);
}

Eigen::Affine3d Scene::getObjectPose(const std::string &name)
{
    auto &world = scene_->getWorldNonConst();
    const auto &obj = world->getObject(name);
    if (obj)
        return obj->shape_poses_[0];

    return Eigen::Affine3d::Identity();
}

bool Scene::attachObject(const std::string &name)
{
    const auto &robot = scene_->getCurrentState().getRobotModel();
    const auto &ee = robot->getEndEffectors();

    // One end-effector
    if (ee.size() == 1)
    {
        const auto &links = ee[0]->getLinkModelNames();
        return attachObject(name, links[0], links);
    }

    return false;
}

bool Scene::attachObject(const std::string &name, const std::string &ee_link,
                         const std::vector<std::string> &touch_links)
{
    auto &world = scene_->getWorldNonConst();
    if (!world->hasObject(name))
    {
        ROS_ERROR("World does not have object `%s`", name.c_str());
        return false;
    }

    auto &robot = scene_->getCurrentStateNonConst();
    const auto &obj = world->getObject(name);

    if (!obj)
    {
        ROS_ERROR("Could not get object `%s`", name.c_str());
        return false;
    }

    if (!world->removeObject(name))
    {
        ROS_ERROR("Could not remove object `%s`", name.c_str());
        return false;
    }

    robot.attachBody(name, obj->shapes_, obj->shape_poses_, touch_links, ee_link);
    return true;
}

bool Scene::detachObject(const std::string &name)
{
    auto &robot = scene_->getCurrentStateNonConst();
    auto &world = scene_->getWorldNonConst();
    auto body = robot.getAttachedBody(name);
    if (!body)
    {
        ROS_ERROR("Robot does not have attached object `%s`", name.c_str());
        return false;
    }

    world->addToObject(name, body->getShapes(), body->getFixedTransforms());
}

bool Scene::toYAMLFile(const std::string &file)
{
    moveit_msgs::PlanningScene msg;
    scene_->getPlanningSceneMsg(msg);

    return IO::messageToYAMLFile(msg, file);
}

bool Scene::fromYAMLFile(const std::string &file)
{

    moveit_msgs::PlanningScene msg;
    if (!IO::YAMLFileToMessage(msg, file))
        return false;

    scene_->setPlanningSceneMsg(msg);
    return true;
}

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

struct LoadIntoMoveIt {
    Eigen::Affine3d robot_offset;
    std::map<std::string, Geometry> objects_with_ori_offsets;
};

void parse_kinbody(LoadIntoMoveIt& load_struct, TiXmlElement *elem, Eigen::Affine3d tf)
{
    if (not elem)
    {
        ROS_ERROR("Element doesn't exist?");
        throw Exception(1, "Element doesn't exist?");
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
            throw Exception(1, "Cannot load file");
        }
        TiXmlHandle hDoc(&doc);
        parse_kinbody(load_struct, hDoc.FirstChildElement("KinBody").Element(), tf * this_tf);

        return;
    }

    TiXmlElement *bodyElem = hElem.FirstChild().Element();
    for (bodyElem; bodyElem; bodyElem = bodyElem->NextSiblingElement())
    {
        if (std::string(bodyElem->Value()) == "Body")
        {
            Geometry g(bodyElem, this_tf * tf);
            ROS_INFO("Pushing back geometry");
            load_struct.objects_with_ori_offsets.insert(std::make_pair(bodyElem->Attribute("name"), g));
        }
    }
}

bool Scene::fromOpenRAVEXMLFile(const std::string &file)
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
            parse_kinbody(load_struct, pElem, load_struct.robot_offset.inverse());
        }
        else
        {
            ROS_INFO("Ignoring elements of value %s", pKey.c_str());
        }
    }
    auto rob_trans = load_struct.robot_offset.translation();
    ROS_INFO("At the end, we found a rob translation of (%f, %f, %f), and we found %zu objects", 
             rob_trans[0], rob_trans[1], rob_trans[2], load_struct.objects_with_ori_offsets.size());

    moveit_msgs::PlanningScene planning_scene;
    for (auto it = load_struct.objects_with_ori_offsets.begin(); it != load_struct.objects_with_ori_offsets.end(); it++)
    {
        planning_scene.is_diff = true;
        planning_scene.world.collision_objects.push_back(
            makeCollisionObject(it->first, it->second));
    }
    scene_->setPlanningSceneMsg(planning_scene);
    return true;
}
