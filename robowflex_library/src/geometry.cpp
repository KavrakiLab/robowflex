#include <geometric_shapes/shape_operations.h>

#include "robowflex.h"

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
  : type_(type), dimensions_(dimensions), resource_("file://" + IO::resolvePath(resource)), shape_(loadShape())
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
