/* Author: Zachary Kingston */

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <geometric_shapes/shape_operations.h>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>
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
        setColor(pair.second, dart::Color::Blue(0.2));
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

void Structure::dumpGraphViz(std::ostream &out, bool standalone)
{
    if (standalone)
        out << "digraph {" << std::endl;

    const std::string &sname = skeleton_->getName();
    for (const auto &node : skeleton_->getBodyNodes())
    {
        const std::string &name = node->getName();
        const std::size_t index = node->getIndexInSkeleton();

        out << sname << "_" << index << "[label=\"" << name << "\"]" << std::endl;

        const auto &joint = node->getParentJoint();
        const std::string &jname = joint->getName();
        const std::string &type = joint->getType();

        const auto &parent = joint->getParentBodyNode();
        if (parent)
        {
            const auto &pindex = parent->getIndexInSkeleton();
            out << sname << "_" << pindex << "->"  //
                << sname << "_" << index           //
                << "[label=\"" << jname << std::endl
                << type << "\"]" << std::endl;
        }
    }

    if (standalone)
        out << "}" << std::endl;
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
    if (mass <= 0)
        mass = 1.;

    inertia.setMass(mass);
    inertia.setMoment(shape->computeInertia(mass));

    if (not inertia.verify(false))
        inertia = dart::dynamics::Inertia(mass);

    body->setInertia(inertia);
    body->setRestitutionCoeff(magic::DEFAULT_RESTITUTION);

    auto *joint = body->getParentJoint();
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

void Structure::addGround(double z, double radius)
{
    const double thickness = 0.01;

    dart::dynamics::WeldJoint::Properties joint;
    joint.mName = "ground";
    joint.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, z - thickness);

    auto ground = makeBox(radius, radius, thickness);
    auto pair = addWeldedFrame(joint, ground);

    setColor(pair.second, dart::Color::Blue(0.2));
}

void Structure::setJoint(const std::string &name, double value)
{
    setJoint(name, Eigen::VectorXd::Constant(1, value));
}

void Structure::setJoint(const std::string &name, const Eigen::Ref<const Eigen::VectorXd> &value)
{
    auto *joint = skeleton_->getJoint(name);
    joint->setPositions(value);
}

bool Structure::solveIK()
{
    auto ik = skeleton_->getIK(true);
    // ik->getSolver()->setTolerance(1e-9);
    // ik->getSolver()->setNumMaxIterations(100);
    return ik->solveAndApply(true);
}

void Structure::setDof(unsigned int index, double value)
{
    skeleton_->getDof(index)->setPosition(value);
}

std::vector<std::string> Structure::getJointNames() const
{
    std::vector<std::string> names;
    for (const auto *joint : skeleton_->getJoints())
        names.emplace_back(joint->getName());

    return names;
}

dart::dynamics::Joint *Structure::getJoint(const std::string &joint_name) const
{
    return skeleton_->getJoint(joint_name);
}

dart::dynamics::BodyNode *Structure::getFrame(const std::string &name) const
{
    return skeleton_->getBodyNode(name);
}

void Structure::reparentFreeFrame(dart::dynamics::BodyNode *child, const std::string &parent)
{
    auto *frame = getFrame(parent);

    RobotPose tf;
    if (frame)
        tf = child->getTransform(frame);
    else
        tf = child->getWorldTransform();

    dart::dynamics::FreeJoint::Properties joint;
    joint.mName = child->getName();
    auto *jt = child->moveTo<dart::dynamics::FreeJoint>(skeleton_, frame, joint);

    setJointParentTransform(joint.mName, tf);
}

void Structure::setJointParentTransform(const std::string &name, const RobotPose &tf)
{
    auto *joint = skeleton_->getJoint(name);
    if (joint == nullptr)
        RBX_ERROR("Cannot find joint named %s to set TF!", name);

    joint->setTransformFromParentBodyNode(tf);
}

void Structure::updateCollisionObject(const std::string &name, const GeometryPtr &geometry,
                                      const robowflex::RobotPose &pose)
{
    auto nodes = skeleton_->getBodyNodes(name);  // Get all nodes with this name
    if (!nodes.empty())
        setJointParentTransform(name, pose);
    else
    {
        dart::dynamics::FreeJoint::Properties joint;
        joint.mName = name;
        joint.mT_ParentBodyToJoint = robowflex::TF::identity();

        auto shape = makeGeometry(geometry);

        auto pair = addFreeFrame(joint, shape);
        setJointParentTransform(name, pose);
        setColor(pair.second, dart::Color::Blue(0.2));
    }
}

dart::dynamics::BodyNode *Structure::getRootFrame() const
{
    return skeleton_->getRootBodyNode();
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

std::shared_ptr<dart::dynamics::BoxShape>
robowflex::darts::makeBox(const Eigen::Ref<const Eigen::Vector3d> &v)
{
    return std::make_shared<dart::dynamics::BoxShape>(v);
}

std::shared_ptr<dart::dynamics::BoxShape> robowflex::darts::makeBox(double x, double y, double z)
{
    return makeBox(Eigen::Vector3d{x, y, z});
}

std::shared_ptr<dart::dynamics::CylinderShape> robowflex::darts::makeCylinder(double radius, double height)
{
    return std::make_shared<dart::dynamics::CylinderShape>(radius, height);
}

std::shared_ptr<dart::dynamics::SphereShape> robowflex::darts::makeSphere(double radius)
{
    return std::make_shared<dart::dynamics::SphereShape>(radius);
}

std::shared_ptr<dart::dynamics::MeshShape> robowflex::darts::makeMesh(const GeometryPtr &geometry)
{
    std::string uri = geometry->getResource();
    Eigen::Vector3d dimensions = geometry->getDimensions();

    if (uri.empty())
    {
        const auto &temp_file_name = ".robowflex_tmp.stl";

        auto shape = std::dynamic_pointer_cast<shapes::Mesh>(geometry->getShape());

        std::vector<char> buffer;
        shapes::writeSTLBinary(shape.get(), buffer);

        std::ofstream out;
        out.open(temp_file_name, std::ofstream::out | std::ofstream::binary);
        out.write(buffer.data(), buffer.size());
        out.close();

        uri = temp_file_name;
    }

    const auto &file = robowflex::IO::resolvePackage(uri);
    const auto &aiscene = dart::dynamics::MeshShape::loadMesh(file);

    auto mesh = std::make_shared<dart::dynamics::MeshShape>(dimensions, aiscene);
    return mesh;
}

std::shared_ptr<dart::dynamics::MeshShape> robowflex::darts::makeArcsegment(double low, double high,
                                                                            double inner_radius,
                                                                            double outer_radius,
                                                                            std::size_t resolution)
{
    if (resolution < 1)
        throw std::runtime_error("Invalid resolution.");

    if (inner_radius > outer_radius)
        throw std::runtime_error("Invalid radii.");

    if (low > high)
        low = high;

    const double interval = high - low;
    const double step = interval / resolution;

    auto *mesh = new aiMesh;
    mesh->mMaterialIndex = (unsigned int)(-1);

    // number of segments, *2 for inner & outer, *2 for front & back
    const std::size_t n_seg = resolution + 1;
    const std::size_t n_v_face = n_seg * 2;
    const std::size_t n_vert = n_v_face * 2;

    const std::size_t n_face = resolution * 2;

    mesh->mNumVertices = n_vert;
    mesh->mVertices = new aiVector3D[n_vert];
    mesh->mNormals = new aiVector3D[n_vert];
    mesh->mColors[0] = new aiColor4D[n_vert];

    const double vx = 0;
    aiVector3D vertex;

    const aiVector3D fnormal(1.0f, 0.0f, 0.0f);
    const aiVector3D bnormal(-1.0f, 0.0f, 0.0f);
    const aiColor4D color(0.0f, 0.0f, 0.0f, 1.0f);

    // Vertices
    for (std::size_t i = 0; i <= resolution; ++i)
    {
        const double theta = low + step * i;

        // Inner Radius
        const double yi = inner_radius * std::cos(theta);
        const double zi = inner_radius * std::sin(theta);
        vertex.Set(vx, yi, zi);

        // front
        const std::size_t ifv = 2 * i;
        mesh->mVertices[ifv] = vertex;
        mesh->mNormals[ifv] = fnormal;
        mesh->mColors[0][ifv] = color;

        // back
        const std::size_t ibv = ifv + n_v_face;
        mesh->mVertices[ibv] = vertex;
        mesh->mNormals[ibv] = bnormal;
        mesh->mColors[0][ibv] = color;

        // Outer Radius
        const double yo = outer_radius * std::cos(theta);
        const double zo = outer_radius * std::sin(theta);
        vertex.Set(vx, yo, zo);

        // front
        const std::size_t ofv = ifv + 1;
        mesh->mVertices[ofv] = vertex;
        mesh->mNormals[ofv] = fnormal;
        mesh->mColors[0][ofv] = color;

        // back
        const std::size_t obv = ofv + n_v_face;
        mesh->mVertices[obv] = vertex;
        mesh->mNormals[obv] = bnormal;
        mesh->mColors[0][obv] = color;
    }

    mesh->mNumFaces = n_face;
    mesh->mFaces = new aiFace[n_face];

    // Faces
    for (std::size_t i = 1; i <= resolution; ++i)
    {
        // front
        const std::size_t ifv = 2 * i;
        const std::size_t ifvp = 2 * (i - 1);
        const std::size_t ofv = ifv + 1;
        const std::size_t ofvp = ifvp + 1;

        aiFace *fface = &mesh->mFaces[i - 1];
        fface->mNumIndices = 4;
        fface->mIndices = new unsigned int[4];
        fface->mIndices[0] = ifv;
        fface->mIndices[1] = ifvp;
        fface->mIndices[2] = ofvp;
        fface->mIndices[3] = ofv;

        // back
        const std::size_t ibv = ifv + n_v_face;
        const std::size_t ibvp = ifvp + n_v_face;
        const std::size_t obv = ofv + n_v_face;
        const std::size_t obvp = ofvp + n_v_face;

        aiFace *bface = &mesh->mFaces[i + resolution - 1];
        bface->mNumIndices = 4;
        bface->mIndices = new unsigned int[4];
        bface->mIndices[0] = obv;
        bface->mIndices[1] = obvp;
        bface->mIndices[2] = ibvp;
        bface->mIndices[3] = ibv;
    }

    auto *node = new aiNode;
    node->mNumMeshes = 1;
    node->mMeshes = new unsigned int[1];
    node->mMeshes[0] = 0;

    auto *scene = new aiScene;
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh *[1];
    scene->mMeshes[0] = mesh;
    scene->mRootNode = node;

    auto shape = std::make_shared<dart::dynamics::MeshShape>(Eigen::Vector3d::Ones(), scene);
    shape->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);

    return shape;
}

void robowflex::darts::setColor(dart::dynamics::BodyNode *node, const Eigen::Vector4d &color)
{
    for (const auto &shape : node->getShapeNodes())
        shape->getVisualAspect()->setColor(color);
}
