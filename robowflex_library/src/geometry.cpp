/* Author: Zachary Kingston */

#include <geometric_shapes/shape_operations.h>

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
    {
        // TODO: Throw
    }

    return type;
}

const std::string &Geometry::ShapeType::toString(Type type)
{
    if (type > MAX)
    {
        // TODO: Throw
    }

    return STRINGS[type];
}

Geometry::Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource)
  : type_(type)
  , dimensions_(dimensions)
  , resource_("file://" + IO::resolvePath(resource))
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

const bool Geometry::contains(const Eigen::Vector3d &point) const
{
    return body_->containsPoint(point[0], point[1], point[2]);
}

Eigen::Vector3d Geometry::sample(const unsigned int attempts) const
{
    Eigen::Vector3d point;
    random_numbers::RandomNumberGenerator rng;

    if (!body_->samplePointInside(rng, attempts, point))
        point = Eigen::Vector3d{0, 0, 0};

    return point;
}

const bool Geometry::isMesh() const
{
    return type_ == ShapeType::MESH;
}

const shape_msgs::SolidPrimitive Geometry::getSolidMsg() const
{
    shapes::ShapeMsg msg;
    if (type_ != ShapeType::MESH)
        shapes::constructMsgFromShape(shape_.get(), msg);

    return boost::get<shape_msgs::SolidPrimitive>(msg);
}

const shape_msgs::Mesh Geometry::getMeshMsg() const
{
    shapes::ShapeMsg msg;
    if (type_ == ShapeType::MESH)
        shapes::constructMsgFromShape(shape_.get(), msg);

    return boost::get<shape_msgs::Mesh>(msg);
}
