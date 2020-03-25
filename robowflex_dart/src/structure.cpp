/* Author: Zachary Kingston */

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include <robowflex_library/scene.h>

#include <robowflex_dart/acm.h>
#include <robowflex_dart/structure.h>

using namespace robowflex::darts;

///
/// Structure
///

Structure::Structure(const std::string &name)
  : name_(name), skeleton_(dart::dynamics::Skeleton::create(name_)), acm_(std::make_shared<ACM>(this))
{
    skeleton_->setSelfCollisionCheck(true);
}

Structure::Structure(const std::string &name, const ScenePtr &scene) : Structure(name)
{
}

const std::string &Structure::getName() const
{
    return name_;
}

ACMPtr Structure::getACM()
{
    return acm_;
}

const ACMPtr &Structure::getACMConst() const
{
    return acm_;
}

void Structure::setSkeleton(const dart::dynamics::SkeletonPtr &skeleton)
{
    skeleton_ = skeleton;
    skeleton_->setName(name_);
}

dart::dynamics::SkeletonPtr &Structure::getSkeleton()
{
    return skeleton_;
}

const dart::dynamics::SkeletonPtr &Structure::getSkeletonConst() const
{
    return skeleton_;
}

void Structure::createShapeNode(dart::dynamics::BodyNode *body, const dart::dynamics::ShapePtr &shape)
{
    body->createShapeNodeWith<dart::dynamics::VisualAspect,     //
                              dart::dynamics::CollisionAspect,  //
                              dart::dynamics::DynamicsAspect>(shape);

    dart::dynamics::Inertia inertia;
    double mass = magic::DEFAULT_DENSITY * shape->getVolume();
    inertia.setMass(mass);
    inertia.setMoment(shape->computeInertia(mass));
    body->setInertia(inertia);

    auto joint = body->getParentJoint();
    if (joint)
        for (std::size_t i = 0; i < joint->getNumDofs(); ++i)
            joint->getDof(i)->setDampingCoefficient(magic::DEFAULT_DAMPING);
}

std::pair<dart::dynamics::RevoluteJoint *, dart::dynamics::BodyNode *>                    //
Structure::addRevoluteFrame(const dart::dynamics::RevoluteJoint::Properties &properties,  //
                            const dart::dynamics::ShapePtr &shape,                        //
                            dart::dynamics::BodyNode *parent)
{
    dart::dynamics::BodyNode::Properties node;
    node.mName = properties.mName;

    auto pair =
        skeleton_->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(parent, properties, node);
    createShapeNode(pair.second, shape);

    pair.first->setPositionLimitEnforced(true);
    return pair;
}

std::pair<dart::dynamics::PrismaticJoint *, dart::dynamics::BodyNode *>                     //
Structure::addPrismaticFrame(const dart::dynamics::PrismaticJoint::Properties &properties,  //
                             const dart::dynamics::ShapePtr &shape,                         //
                             dart::dynamics::BodyNode *parent)
{
    dart::dynamics::BodyNode::Properties node;
    node.mName = properties.mName;

    auto pair =
        skeleton_->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(parent, properties, node);
    createShapeNode(pair.second, shape);

    pair.first->setPositionLimitEnforced(true);
    return pair;
}

std::pair<dart::dynamics::FreeJoint *, dart::dynamics::BodyNode *>                //
Structure::addFreeFrame(const dart::dynamics::FreeJoint::Properties &properties,  //
                        const dart::dynamics::ShapePtr &shape,                    //
                        dart::dynamics::BodyNode *parent)
{
    dart::dynamics::BodyNode::Properties node;
    node.mName = properties.mName;

    auto pair = skeleton_->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(parent, properties, node);
    createShapeNode(pair.second, shape);

    pair.first->setPositionLimitEnforced(true);
    return pair;
}

std::pair<dart::dynamics::WeldJoint *, dart::dynamics::BodyNode *>                  //
Structure::addWeldedFrame(const dart::dynamics::WeldJoint::Properties &properties,  //
                          const dart::dynamics::ShapePtr &shape,                    //
                          dart::dynamics::BodyNode *parent)
{
    dart::dynamics::BodyNode::Properties node;
    node.mName = properties.mName;

    auto pair = skeleton_->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(parent, properties, node);
    createShapeNode(pair.second, shape);

    return pair;
}

void Structure::addGround(double z)
{
    const double thickness = 0.01;

    dart::dynamics::WeldJoint::Properties joint;
    joint.mName = "ground";
    // joint.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    joint.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, z - thickness);

    auto ground = makeBox(10, 10, thickness);
    auto pair = addWeldedFrame(joint, ground);

    setColor(pair.second, dart::Color::Blue(0.2));
}

dart::dynamics::ShapePtr robowflex::darts::makeBox(const Eigen::Ref<const Eigen::Vector3d> &v)
{
    return std::make_shared<dart::dynamics::BoxShape>(v);
}

dart::dynamics::ShapePtr robowflex::darts::makeBox(double x, double y, double z)
{
    return makeBox(Eigen::Vector3d{x, y, z});
}

void robowflex::darts::setColor(dart::dynamics::BodyNode *node, const Eigen::Vector4d &color)
{
    for (auto &shape : node->getShapeNodes())
        shape->getVisualAspect()->setColor(color);
}
