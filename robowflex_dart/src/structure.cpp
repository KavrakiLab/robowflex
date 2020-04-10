/* Author: Zachary Kingston */

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <geometric_shapes/shape_operations.h>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/MeshShape.hpp>

#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include <robowflex_library/geometry.h>
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
    const auto objects = scene->getCollisionObjects();

    for (const auto &object : objects)
    {
        const auto &geometry = scene->getObjectGeometry(object);
        const auto &pose = scene->getObjectPose(object);

        dart::dynamics::FreeJoint::Properties joint;
        joint.mName = object;
        joint.mT_ParentBodyToJoint = pose;

        auto shape = makeGeometry(geometry);

        auto pair = addFreeFrame(joint, shape);
        setColor(pair.second, dart::Color::Blue(0.2));
    }

    auto acm = scene->getACM();
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

StructurePtr Structure::cloneStructure(const std::string &newName) const
{
    auto structure = std::make_shared<Structure>(newName);
    structure->setSkeleton(skeleton_->cloneSkeleton(newName));

    for (const auto &pair : acm_->getDisabledPairsConst())
        structure->getACM()->disableCollision(pair.first, pair.second);

    return structure;
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
    if (not shape)
        return;

    body->createShapeNodeWith<dart::dynamics::VisualAspect,     //
                              dart::dynamics::CollisionAspect,  //
                              dart::dynamics::DynamicsAspect>(shape);

    dart::dynamics::Inertia inertia;
    double mass = magic::DEFAULT_DENSITY * shape->getVolume();
    inertia.setMass(mass);
    inertia.setMoment(shape->computeInertia(mass));
    body->setInertia(inertia);
    body->setRestitutionCoeff(magic::DEFAULT_RESTITUTION);

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

void Structure::setJoint(const std::string &name, double value)
{
    setJoint(name, Eigen::VectorXd::Constant(1, value));
}

void Structure::setJoint(const std::string &name, const Eigen::Ref<const Eigen::VectorXd> &value)
{
    auto joint = skeleton_->getJoint(name);
    joint->setPositions(value);
}

bool Structure::solveIK()
{
    auto ik = skeleton_->getIK(true);
    // ik->getSolver()->setTolerance(1e-9);
    // ik->getSolver()->setNumMaxIterations(100);
    return ik->solveAndApply(true);
}

dart::dynamics::ShapePtr robowflex::darts::makeGeometry(const GeometryPtr &geometry)
{
    const auto &dimensions = geometry->getDimensions();

    switch (geometry->getType())
    {
        case Geometry::ShapeType::BOX:
            return makeBox(dimensions);
        case Geometry::ShapeType::SPHERE:
            return makeSphere(dimensions[0]);
        case Geometry::ShapeType::CYLINDER:
            return makeCylinder(dimensions[0], dimensions[1]);
        case Geometry::ShapeType::MESH:
            return makeMesh(geometry);
        default:
            break;
    }

    return nullptr;
}

dart::dynamics::ShapePtr robowflex::darts::makeBox(const Eigen::Ref<const Eigen::Vector3d> &v)
{
    return std::make_shared<dart::dynamics::BoxShape>(v);
}

dart::dynamics::ShapePtr robowflex::darts::makeBox(double x, double y, double z)
{
    return makeBox(Eigen::Vector3d{x, y, z});
}

dart::dynamics::ShapePtr robowflex::darts::makeCylinder(double radius, double height)
{
    return std::make_shared<dart::dynamics::CylinderShape>(radius, height);
}

dart::dynamics::ShapePtr robowflex::darts::makeSphere(double radius)
{
    return std::make_shared<dart::dynamics::SphereShape>(radius);
}

dart::dynamics::ShapePtr robowflex::darts::makeMesh(const GeometryPtr &geometry)
{
    static Assimp::Importer importer_;

    auto shape = std::dynamic_pointer_cast<shapes::Mesh>(geometry->getShape());
    std::vector<char> buffer;
    shapes::writeSTLBinary(shape.get(), buffer);

    auto aiscene = importer_.ReadFileFromMemory(buffer.data(), buffer.size(),
                                                aiProcessPreset_TargetRealtime_MaxQuality, "stl");

    return std::make_shared<dart::dynamics::MeshShape>(geometry->getDimensions(), aiscene);
}

void robowflex::darts::setColor(dart::dynamics::BodyNode *node, const Eigen::Vector4d &color)
{
    for (auto &shape : node->getShapeNodes())
        shape->getVisualAspect()->setColor(color);
}
