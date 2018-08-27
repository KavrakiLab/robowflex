/* Author: Zachary Kingston */

#include <geometric_shapes/shape_operations.h>

#include <robowflex_library/util.h>
#include <robowflex_library/io.h>
#include <robowflex_library/geometry.h>

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

GeometryPtr Geometry::makeCylinder(double radius, double length)
{
    return std::make_shared<Geometry>(ShapeType::CYLINDER, Eigen::Vector3d(radius, length, 0));
}

GeometryPtr Geometry::makeCone(double radius, double length)
{
    return std::make_shared<Geometry>(ShapeType::CONE, Eigen::Vector3d(radius, length, 0));
}

GeometryPtr Geometry::makeMesh(const std::string &resource, const Eigen::Vector3d &scale)
{
    return std::make_shared<Geometry>(ShapeType::MESH, scale, resource);
}

Geometry::Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource)
  : type_(type)
  , dimensions_(dimensions)
  , resource_((resource.empty()) ? "" : "file://" + IO::resolvePath(resource))
  , shape_(loadShape())
  , body_(loadBody())
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
            return shapes::createMeshFromResource(resource_, dimensions_);
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

const Eigen::Vector3d &Geometry::getDimensions() const
{
    return dimensions_;
}
