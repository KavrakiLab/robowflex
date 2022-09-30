/* Author: Zachary Kingston, Constantinos Chamzas */

#include <geometric_shapes/shape_operations.h>

#include <moveit/robot_model/aabb.h>

#include <robowflex_util/filesystem.h>
#include <robowflex_moveit/core/geometry.h>

using namespace robowflex;

MoveItGeometryPtr MoveItGeometry::makeSphere(double radius)
{
    return std::make_shared<MoveItGeometry>(ShapeType::SPHERE, Eigen::Vector3d(radius, 0, 0));
}

MoveItGeometryPtr MoveItGeometry::makeBox(double x, double y, double z)
{
    return std::make_shared<MoveItGeometry>(ShapeType::BOX, Eigen::Vector3d(x, y, z));
}

MoveItGeometryPtr MoveItGeometry::makeBox(const Eigen::Vector3d &dimensions)
{
    return makeBox(dimensions[0], dimensions[1], dimensions[2]);
}

MoveItGeometryPtr MoveItGeometry::makeCylinder(double radius, double length)
{
    return std::make_shared<MoveItGeometry>(ShapeType::CYLINDER, Eigen::Vector3d(radius, length, 0));
}

MoveItGeometryPtr MoveItGeometry::makeCone(double radius, double length)
{
    return std::make_shared<MoveItGeometry>(ShapeType::CONE, Eigen::Vector3d(radius, length, 0));
}

MoveItGeometryPtr MoveItGeometry::makeSolidPrimitive(const shape_msgs::SolidPrimitive &msg)
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

MoveItGeometryPtr MoveItGeometry::makeMesh(const std::string &resource, const Eigen::Vector3d &scale)
{
    return std::make_shared<MoveItGeometry>(ShapeType::MESH, scale, resource);
}

MoveItGeometryPtr MoveItGeometry::makeMesh(const std::vector<Eigen::Vector3d> &vertices)
{
    return std::make_shared<MoveItGeometry>(ShapeType::MESH, Eigen::Vector3d::Ones(), "", vertices);
}

MoveItGeometry::MoveItGeometry(ShapeType::Type type, const Eigen::Vector3d &dimensions,
                               const std::string &resource, const std::vector<Eigen::Vector3d> &vertices)
  : Geometry(type, dimensions, resource, vertices), shape_(loadShape()), body_(loadBody())
{
}

MoveItGeometry::MoveItGeometry(const Geometry &geo)
  : MoveItGeometry(geo.getType(), geo.getDimensions(), geo.getResource(), geo.getVertices())
{
}

MoveItGeometry::MoveItGeometry(const shapes::Shape &shape)
  : Geometry(  //
        [&shape] {
            switch (shape.type)
            {
                case shapes::ShapeType::BOX:
                    return ShapeType::BOX;
                case shapes::ShapeType::SPHERE:
                    return ShapeType::SPHERE;
                case shapes::ShapeType::CYLINDER:
                    return ShapeType::CYLINDER;
                case shapes::ShapeType::CONE:
                    return ShapeType::CONE;
                case shapes::ShapeType::MESH:
                    return ShapeType::MESH;
                default:
                    throw std::runtime_error("Invalid type for geometry.");
            }
        }(),
        [&shape] {
            switch (shape.type)
            {
                case shapes::ShapeType::BOX:
                {
                    const auto &box = static_cast<const shapes::Box &>(shape);
                    return Eigen::Vector3d{box.size[0], box.size[1], box.size[2]};
                }
                case shapes::ShapeType::SPHERE:
                {
                    const auto &sphere = static_cast<const shapes::Sphere &>(shape);
                    return Eigen::Vector3d{sphere.radius, 0, 0};
                }
                case shapes::ShapeType::CYLINDER:
                {
                    const auto &cylinder = static_cast<const shapes::Cylinder &>(shape);
                    return Eigen::Vector3d{cylinder.radius, cylinder.length, 0};
                }
                case shapes::ShapeType::CONE:
                {
                    const auto &cone = static_cast<const shapes::Cone &>(shape);
                    return Eigen::Vector3d{cone.radius, cone.length, 0};
                }
                case shapes::ShapeType::MESH:
                    return Eigen::Vector3d{1., 1., 1.};
                default:
                    throw std::runtime_error("Invalid type for geometry.");
            }
        }(),
        "",
        [&shape]() {
            if (shape.type != shapes::ShapeType::MESH)
                return std::vector<Eigen::Vector3d>{};

            const auto &mesh = static_cast<const shapes::Mesh &>(shape);

            std::vector<Eigen::Vector3d> vertices;

            for (std::size_t i = 0; i < mesh.triangle_count; ++i)
            {
                unsigned int *triangle = &mesh.triangles[3 * i];
                for (std::size_t j = 0; j < 3; ++j)
                {
                    double *vertex = &mesh.vertices[3 * triangle[j]];
                    vertices.emplace_back(vertex[0], vertex[1], vertex[2]);
                }
            }

            return vertices;
        }())
{
}

MoveItGeometry::MoveItGeometry(const shape_msgs::SolidPrimitive &msg)
  : MoveItGeometry(*shapes::constructShapeFromMsg(msg))
{
}

MoveItGeometry::MoveItGeometry(const shape_msgs::Mesh &msg)
  : MoveItGeometry(*shapes::constructShapeFromMsg(msg))
{
}

shapes::Shape *MoveItGeometry::loadShape() const
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
            {
                EigenSTL::vector_Vector3d vertices_tmp(vertices_.begin(), vertices_.end());
                return shapes::createMeshFromVertices(vertices_tmp);
            }
            else
                throw std::runtime_error(                                //
                    resource_.empty() ?                                  //
                        "No vertices/resource specified for the mesh" :  //
                        "Both vertices/resource specified for the mesh");
            break;

        default:
            break;
    }

    return nullptr;
}

bodies::Body *MoveItGeometry::loadBody() const
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

bool MoveItGeometry::contains(const Eigen::Vector3d &point) const
{
    return body_->containsPoint(point[0], point[1], point[2]);
}

std::pair<bool, Eigen::Vector3d> MoveItGeometry::sample(const unsigned int attempts) const
{
    bool success;
    Eigen::Vector3d point;
    random_numbers::RandomNumberGenerator rng;

    if (!(success = body_->samplePointInside(rng, attempts, point)))
        point = Eigen::Vector3d{0, 0, 0};

    return std::make_pair(success, point);
}

const shape_msgs::SolidPrimitive MoveItGeometry::getSolidMsg() const
{
    shapes::ShapeMsg msg;
    if (!isMesh())
        shapes::constructMsgFromShape(shape_.get(), msg);

    return boost::get<shape_msgs::SolidPrimitive>(msg);
}

const shape_msgs::Mesh MoveItGeometry::getMeshMsg() const
{
    shapes::ShapeMsg msg;
    if (isMesh())
        shapes::constructMsgFromShape(shape_.get(), msg);

    return boost::get<shape_msgs::Mesh>(msg);
}

const shapes::ShapePtr &MoveItGeometry::getShape() const
{
    return shape_;
}

const bodies::BodyPtr &MoveItGeometry::getBody() const
{
    return body_;
}

Eigen::AlignedBox3d MoveItGeometry::getAABB(const RobotPose &pose) const
{
    moveit::core::AABB aabb;

    const auto &extents = shapes::computeShapeExtents(shape_.get());
    aabb.extendWithTransformedBox(pose, extents);

    return aabb;
}
