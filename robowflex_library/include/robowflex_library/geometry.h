#ifndef ROBOWFLEX_GEOMETRY_
#define ROBOWFLEX_GEOMETRY_

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    class Geometry
    {
    public:
        class ShapeType
        {
        public:
            enum Type
            {
                BOX = 0,
                SPHERE = 1,
                CYLINDER = 2,
                CONE = 3,
                MESH = 4
            };

            static const unsigned int MAX;
            static const std::vector<std::string> STRINGS;

            static Type toType(const std::string &str);
            static const std::string &toString(Type type);
        };

        Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource = "");

        Geometry(const Geometry &) = delete;             // non construction-copyable
        Geometry &operator=(const Geometry &) = delete;  // non copyable

        const bool contains(const Eigen::Vector3d &point) const;
        Eigen::Vector3d sample(const unsigned int attempts = 50) const;

        const bool isMesh() const;

        const shape_msgs::SolidPrimitive getSolidMsg() const;
        const shape_msgs::Mesh getMeshMsg() const;

        const shapes::ShapePtr &getShape() const
        {
            return shape_;
        }

        const bodies::BodyPtr &getBody() const
        {
            return body_;
        }

    private:
        shapes::Shape *loadShape() const;
        bodies::Body *loadBody() const;

        ShapeType::Type type_{ShapeType::Type::BOX};                 // Geometry Type.
        std::string resource_{""};                                   // Resource locator for MESH types.
        const Eigen::Vector3d dimensions_{Eigen::Vector3d::Ones()};  // Dimensions to scale geometry along
                                                                     // axes.
        const shapes::ShapePtr shape_{nullptr};                      // Loaded shape.
        const bodies::BodyPtr body_{nullptr};                        // Body operation.
    };

    namespace TF
    {
        Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3 &msg);
        geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d &vector);
        Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose &msg);
        geometry_msgs::Pose poseEigenToMsg(const Eigen::Affine3d &pose);
        Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion &msg);
        geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond &quaternion);

        moveit_msgs::BoundingVolume getBoundingVolume(const Eigen::Affine3d &poses,
                                                      const Geometry &geometries);
        moveit_msgs::PositionConstraint getPositionConstraint(const std::string &ee_name,
                                                              const std::string &base_name,
                                                              const Eigen::Affine3d &poses,
                                                              const Geometry &geometries);
        moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string &ee_name,
                                                                    const std::string &base_name,
                                                                    const Eigen::Quaterniond &orientation,
                                                                    const Eigen::Vector3d &tolerances);

        Eigen::Quaterniond sampleOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances);
    }  // namespace TF
}  // namespace robowflex

#endif
