/* Author: Zachary Kingston */

#include <robowflex_moveit/core/geometry.h>
#include <robowflex_moveit/core/scene.h>

#include <robowflex_dart/acm.h>

#include <robowflex_dart_moveit/scene_conversions.h>

using namespace robowflex::darts;

StructurePtr conversions::fromMoveItScene(const std::string &name, const robowflex::SceneConstPtr &scene)
{
    auto structure = std::make_shared<Structure>(name);
    auto &skeleton = structure->getSkeleton();

    dart::dynamics::WeldJoint::Properties properties;
    dart::dynamics::BodyNode::Properties node;

    node.mName = properties.mName = "root";
    properties.mT_ParentBodyToJoint = robowflex::TF::identity();
    const auto &pair =
        skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr, properties, node);
    const auto &root = pair.second;

    const auto &objects = scene->getCollisionObjects();
    for (const auto &object : objects)
    {
        const auto &geometry = scene->getObjectGeometry(object);
        const auto &pose = scene->getObjectPose(object);

        dart::dynamics::FreeJoint::Properties joint;
        joint.mName = object;
        joint.mT_ParentBodyToJoint = robowflex::TF::identity();

        auto shape = makeGeometry(geometry);

        auto pair = structure->addFreeFrame(joint, shape, root);
        structure->setJointParentTransform(object, pose);
        setColor(pair.second, geometry->getColor());
    }

    const auto &scene_acm = scene->getACMConst();
    std::vector<std::string> names;
    scene_acm.getAllEntryNames(names);

    auto acm = structure->getACM();

    collision_detection::AllowedCollision::Type type;
    for (const auto &name1 : names)
        for (const auto &name2 : names)
            if (scene_acm.getEntry(name1, name2, type))
            {
                if (type == collision_detection::AllowedCollision::NEVER)
                    acm->enableCollision(name1, name2);
                else if (type == collision_detection::AllowedCollision::ALWAYS)
                    acm->disableCollision(name1, name2);
            }

    return structure;
}
