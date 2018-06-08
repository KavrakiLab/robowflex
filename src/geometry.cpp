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

Geometry::Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource,
                   const Eigen::Affine3d &offset)
  : type_(type)
  , dimensions_(dimensions)
  , offset_(offset)
  , resource_(IO::resolvePath(resource))
  , shape_(std::move(loadShape()))
{
}

std::shared_ptr<shapes::Shape> Geometry::loadShape() const
{
    switch (type_)
    {
        case ShapeType::BOX:
            return std::make_shared<shapes::Box>(dimensions_[0], dimensions_[1], dimensions_[2]);
            break;

        case ShapeType::SPHERE:
            return std::make_shared<shapes::Sphere>(dimensions_[0]);
            break;

        case ShapeType::CYLINDER:
            return std::make_shared<shapes::Cylinder>(dimensions_[0], dimensions_[1]);
            break;

        case ShapeType::CONE:
            return std::make_shared<shapes::Cone>(dimensions_[0], dimensions_[1]);
            break;

        case ShapeType::MESH:
            return std::shared_ptr<shapes::Mesh>(shapes::createMeshFromResource(resource_, dimensions_));
            break;

        default:
            // TODO:throw
            break;
    }
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
