/* Author: Zachary Kingston, Constantinos Chamzas */

#include <geometric_shapes/shape_operations.h>

#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/util.h>

using namespace robowflex;

const unsigned int Geometry::ShapeType::MAX = (unsigned int)Geometry::ShapeType::MESH + 1;
const std::vector<std::string> Geometry::ShapeType::STRINGS({"box", "sphere", "cylinder", "cone", "mesh"});

Geometry::ShapeType::Type Geometry::ShapeType::toType(const std::string &str)
{
    std::string lower(str);
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    Type type;

    unsigned int i = 0;
    for (; i < MAX; ++i)
        if (STRINGS[i].compare(lower) == 0)
        {
            type = (Type)i;
            break;
        }

    if (i == MAX)
        throw Exception(1, "Invalid type for geometry.");

    return type;
}

const std::string &Geometry::ShapeType::toString(Type type)
{
    if (type > MAX)
        throw Exception(1, "Invalid type for geometry.");

    return STRINGS[type];
}

GeometryPtr Geometry::makeSphere(double radius)
{
    return std::make_shared<Geometry>(ShapeType::SPHERE, Eigen::Vector3d(radius, 0, 0));
}

GeometryPtr Geometry::makeBox(double x, double y, double z)
{
    return std::make_shared<Geometry>(ShapeType::BOX, Eigen::Vector3d(x, y, z));
}

GeometryPtr Geometry::makeBox(const Eigen::Vector3d &dimensions)
{
    return makeBox(dimensions[0], dimensions[1], dimensions[2]);
}

GeometryPtr Geometry::makeCylinder(double radius, double length)
{
    return std::make_shared<Geometry>(ShapeType::CYLINDER, Eigen::Vector3d(radius, length, 0));
}

GeometryPtr Geometry::makeCone(double radius, double length)
{
    return std::make_shared<Geometry>(ShapeType::CONE, Eigen::Vector3d(radius, length, 0));
}

GeometryPtr Geometry::makeSolidPrimitive(const shape_msgs::SolidPrimitive &msg)
{
    using sm = shape_msgs::SolidPrimitive;

    switch (msg.type)
    {
        case sm::BOX:
            return makeBox(msg.dimensions[sm::BOX_X],  //
                           msg.dimensions[sm::BOX_Y],  //
                           msg.dimensions[sm::BOX_Z]);
        case sm::SPHERE:
            return makeSphere(msg.dimensions[sm::SPHERE_RADIUS]);
        case sm::CYLINDER:
            return makeCylinder(msg.dimensions[sm::CYLINDER_RADIUS],  //
                                msg.dimensions[sm::CYLINDER_HEIGHT]);
        case sm::CONE:
            return makeCone(msg.dimensions[sm::CONE_RADIUS],  //
                            msg.dimensions[sm::CONE_HEIGHT]);
        default:
            return nullptr;
    }
}

GeometryPtr Geometry::makeMesh(const std::string &resource, const Eigen::Vector3d &scale)
{
    return std::make_shared<Geometry>(ShapeType::MESH, scale, resource);
}

GeometryPtr Geometry::makeMesh(const EigenSTL::vector_Vector3d &vertices)
{
    return std::make_shared<Geometry>(ShapeType::MESH, Eigen::Vector3d::Ones(), "", vertices);
}

Geometry::Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource,
                   const EigenSTL::vector_Vector3d &vertices)
  : type_(type)
  , dimensions_(dimensions)
  , vertices_(vertices)
  , resource_((resource.empty()) ? "" : IO::resolvePath(resource))
  , shape_(loadShape())
  , body_(loadBody())
{
}

Geometry::Geometry(const shapes::Shape &shape)
{
    switch (shape.type)
    {
        case shapes::ShapeType::BOX:
        {
            type_ = ShapeType::BOX;
            const auto &box = static_cast<const shapes::Box &>(shape);
            dimensions_ = Eigen::Vector3d{box.size[0], box.size[1], box.size[2]};
            shape_.reset(loadShape());
            break;
        }
        case shapes::ShapeType::SPHERE:
        {
            type_ = ShapeType::SPHERE;
            const auto &sphere = static_cast<const shapes::Sphere &>(shape);
            dimensions_ = Eigen::Vector3d{sphere.radius, 0, 0};
            shape_.reset(loadShape());
            break;
        }
        case shapes::ShapeType::CYLINDER:
        {
            type_ = ShapeType::CYLINDER;
            const auto &cylinder = static_cast<const shapes::Cylinder &>(shape);
            dimensions_ = Eigen::Vector3d{cylinder.radius, cylinder.length, 0};
            shape_.reset(loadShape());
            break;
        }
        case shapes::ShapeType::CONE:
        {
            type_ = ShapeType::CONE;
            const auto &cone = static_cast<const shapes::Cone &>(shape);
            dimensions_ = Eigen::Vector3d{cone.radius, cone.length, 0};
            shape_.reset(loadShape());
            break;
        }
        case shapes::ShapeType::MESH:
        {
            type_ = ShapeType::MESH;
            const auto &mesh = static_cast<const shapes::Mesh &>(shape);
            shape_.reset(mesh.clone());
            break;
        }
        default:
            throw Exception(1, "Invalid type for geometry.");
    }

    body_.reset(loadBody());
}

Geometry::Geometry(const shape_msgs::SolidPrimitive &msg) : Geometry(*shapes::constructShapeFromMsg(msg))
{
}

Geometry::Geometry(const shape_msgs::Mesh &msg) : Geometry(*shapes::constructShapeFromMsg(msg))
{
}

shapes::Shape *Geometry::loadShape() const
{
    switch (type_)
    {
        case ShapeType::BOX:
            return new shapes::Box(dimensions_[0], dimensions_[1], dimensions_[2]);
            break;

        case ShapeType::SPHERE:
            return new shapes::Sphere(dimensions_[0]);
            break;

        case ShapeType::CYLINDER:
            return new shapes::Cylinder(dimensions_[0], dimensions_[1]);
            break;

        case ShapeType::CONE:
            return new shapes::Cone(dimensions_[0], dimensions_[1]);
            break;

        case ShapeType::MESH:
            if (!resource_.empty() && vertices_.empty())
                return shapes::createMeshFromResource("file://" + resource_, dimensions_);
            else if (resource_.empty() && !vertices_.empty())
                return shapes::createMeshFromVertices(vertices_);
            else
                throw Exception(1, resource_.empty() ? "No vertices/resource specified for the mesh" :
                                                       "Both vertices/resource specified for the mesh");
            break;

        default:
            break;
    }

    return nullptr;
}

bodies::Body *Geometry::loadBody() const
{
    switch (type_)
    {
        case ShapeType::BOX:
            return new bodies::Box(shape_.get());
            break;

        case ShapeType::SPHERE:
            return new bodies::Sphere(shape_.get());
            break;

        case ShapeType::CYLINDER:
            return new bodies::Cylinder(shape_.get());
            break;

        case ShapeType::MESH:
            return new bodies::ConvexMesh(shape_.get());
            break;

        case ShapeType::CONE:
        default:
            break;
    }

    return nullptr;
}

bool Geometry::contains(const Eigen::Vector3d &point) const
{
    return body_->containsPoint(point[0], point[1], point[2]);
}

std::pair<bool, Eigen::Vector3d> Geometry::sample(const unsigned int attempts) const
{
    bool success;
    Eigen::Vector3d point;
    random_numbers::RandomNumberGenerator rng;

    if (!(success = body_->samplePointInside(rng, attempts, point)))
        point = Eigen::Vector3d{0, 0, 0};

    return std::make_pair(success, point);
}
bool Geometry::isMesh() const
{
    return type_ == ShapeType::MESH;
}

const shape_msgs::SolidPrimitive Geometry::getSolidMsg() const
{
    shapes::ShapeMsg msg;
    if (!isMesh())
        shapes::constructMsgFromShape(shape_.get(), msg);

    return boost::get<shape_msgs::SolidPrimitive>(msg);
}

const shape_msgs::Mesh Geometry::getMeshMsg() const
{
    shapes::ShapeMsg msg;
    if (isMesh())
        shapes::constructMsgFromShape(shape_.get(), msg);

    return boost::get<shape_msgs::Mesh>(msg);
}

const shapes::ShapePtr &Geometry::getShape() const
{
    return shape_;
}

const bodies::BodyPtr &Geometry::getBody() const
{
    return body_;
}

Geometry::ShapeType::Type Geometry::getType() const
{
    return type_;
}

const std::string &Geometry::getResource() const
{
    return resource_;
}

const EigenSTL::vector_Vector3d &Geometry::getVertices() const
{
    return vertices_;
}

const Eigen::Vector3d &Geometry::getDimensions() const
{
    return dimensions_;
}

Eigen::AlignedBox3d Geometry::getAABB(const RobotPose &pose) const
{
    moveit::core::AABB aabb;

    const auto &extents = shapes::computeShapeExtents(shape_.get());
    aabb.extendWithTransformedBox(pose, extents);

    return aabb;
}
