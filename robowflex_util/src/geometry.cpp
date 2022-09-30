/* Author: Zachary Kingston, Constantinos Chamzas */

#include <robowflex_util/random.h>
#include <robowflex_util/geometry.h>
#include <robowflex_util/filesystem.h>

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
        throw std::runtime_error("Invalid type for geometry.");

    return type;
}

const std::string &Geometry::ShapeType::toString(Type type)
{
    if (type > MAX)
        throw std::runtime_error("Invalid type for geometry.");

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

GeometryPtr Geometry::makeMesh(const std::string &resource, const Eigen::Vector3d &scale)
{
    return std::make_shared<Geometry>(ShapeType::MESH, scale, resource);
}

GeometryPtr Geometry::makeMesh(const std::vector<Eigen::Vector3d> &vertices)
{
    return std::make_shared<Geometry>(ShapeType::MESH, Eigen::Vector3d::Ones(), "", vertices);
}

Geometry::Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource,
                   const std::vector<Eigen::Vector3d> &vertices)
  : type_(type)
  , dimensions_(dimensions)
  , vertices_(vertices)
  , resource_((resource.empty()) ? "" : IO::resolvePath(resource))
{
    setFromColorMap(RNG::uniform01());
    setAlpha(1.);
}

bool Geometry::contains(const Eigen::Vector3d &point) const
{
    throw std::runtime_error("Not implemented.");
}

std::pair<bool, Eigen::Vector3d> Geometry::sample(const unsigned int attempts) const
{
    throw std::runtime_error("Not implemented.");
}

bool Geometry::isMesh() const
{
    return type_ == ShapeType::MESH;
}

Geometry::ShapeType::Type Geometry::getType() const
{
    return type_;
}

const std::string &Geometry::getResource() const
{
    return resource_;
}

const std::vector<Eigen::Vector3d> &Geometry::getVertices() const
{
    return vertices_;
}

const Eigen::Vector3d &Geometry::getDimensions() const
{
    return dimensions_;
}

Eigen::AlignedBox3d Geometry::getAABB(const RobotPose &pose) const
{
    throw std::runtime_error("Not implemented.");
}

void Geometry::setColor(const Eigen::Vector4d &color)
{
    color_ = color;
}

void Geometry::setFromColorMap(double v, const color::ColorMap &colormap)
{
    colormap(v, color_);
}

void Geometry::setRedColor(double v)
{
    color_[0] = v;
}

void Geometry::setBlueColor(double v)
{
    color_[2] = v;
}

void Geometry::setGreenColor(double v)
{
    color_[1] = v;
}

void Geometry::setAlpha(double v)
{
    color_[3] = v;
}

const Eigen::Vector4d &Geometry::getColor() const
{
    return color_;
}
