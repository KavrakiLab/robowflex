#include <robowflex_moveit/core/geometry.h>
#include <robowflex_moveit/core/scene.h>

Structure::Structure(const std::string &name, const SceneConstPtr &scene) : Structure(name)
{
    dart::dynamics::WeldJoint::Properties properties;
    dart::dynamics::BodyNode::Properties node;

    node.mName = properties.mName = "root";
    properties.mT_ParentBodyToJoint = robowflex::TF::identity();
    const auto &pair =
        skeleton_->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr, properties, node);
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

        auto pair = addFreeFrame(joint, shape, root);
        setJointParentTransform(object, pose);
        setColor(pair.second, geometry->getColor());
    }

    const auto &acm = scene->getACMConst();
    std::vector<std::string> names;
    acm.getAllEntryNames(names);

    collision_detection::AllowedCollision::Type type;
    for (const auto &name1 : names)
        for (const auto &name2 : names)
            if (acm.getEntry(name1, name2, type))
            {
                if (type == collision_detection::AllowedCollision::NEVER)
                    acm_->enableCollision(name1, name2);
                else if (type == collision_detection::AllowedCollision::ALWAYS)
                    acm_->disableCollision(name1, name2);
            }
}
