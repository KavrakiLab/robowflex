/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_GEOMETRY_
#define ROBOWFLEX_GEOMETRY_

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /** \endcond */

    /** \class robowflex::GeometryPtr
        \brief A shared pointer wrapper for robowflex::Geometry. */

    /** \class robowflex::GeometryConstPtr
        \brief A const shared pointer wrapper for robowflex::Geometry. */

    /** \brief A class that manages both solid and mesh geometry for various parts of the motion planning
     *  system.
     */
    class Geometry
    {
    public:
        /** \brief Supported shape types.
         */
        class ShapeType
        {
        public:
            enum Type
            {
                BOX = 0,       ///< Solid primitive box. Uses three dimensions (x, y, z).
                SPHERE = 1,    ///< Solid primitive sphere. Uses one dimension (radius).
                CYLINDER = 2,  ///< Solid primitive cylinder. Uses two dimensions (height, radius).
                CONE = 3,      ///< Solid primitive cone. Uses two dimensions (height, radius).
                MESH = 4       ///< Mesh. Dimensions scale along x, y, z.
            };

            static const unsigned int MAX;                  ///< Maximum value of ShapeType.
            static const std::vector<std::string> STRINGS;  ///< Mapping of ShapeType to string.

            static Type toType(const std::string &str);     ///< Converts a string into a ShapeType.
            static const std::string &toString(Type type);  ///< Converts a ShapeType to its string.
        };

        /** \brief Constructor.
         *  Builds and loads the specified geometry.
         *  \param[in] type Type of the geometry to create.
         *  \param[in] dimensions Dimensions of the geometry to load.
         *  \param[in] resource If \a type is ShapeType::MESH, then resource must be specified as the mesh
         *                      file to load.
         */
        Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource = "");

        // non-copyable
        Geometry(const Geometry &) = delete;
        Geometry &operator=(const Geometry &) = delete;

        /** \brief Checks if the geometry contains a \a point.
         *  \param[in] point The point to check, in the geometry's frame.
         *  \return True if the geometry contains the point, false otherwise.
         */
        const bool contains(const Eigen::Vector3d &point) const;

        /** \brief Tries to sample a point in the geometry.
         *  \param[in] attempts Number of attempts to sample.
         *  \return The sampled point, or the 0 vector on failure (TODO: fix)
         */
        Eigen::Vector3d sample(const unsigned int attempts = 50) const;

        /** \brief Checks if the geometry is a mesh geometry.
         *  \return True if the \a type_ is a mesh (ShapeType::MESH).
         */
        const bool isMesh() const;

        /** \brief Gets the message form of solid primitive geometry (all but ShapeType::MESH).
         *  \return The message.
         */
        const shape_msgs::SolidPrimitive getSolidMsg() const;

        /** \brief Gets the message form of mesh geometry.
         *  \return The message.
         */
        const shape_msgs::Mesh getMeshMsg() const;

        /** \brief Gets the underlying shape.
         *  \return The shape.
         */
        const shapes::ShapePtr &getShape() const
        {
            return shape_;
        }

        /** \brief Gets the underlying body.
         *  \return The body.
         */
        const bodies::BodyPtr &getBody() const
        {
            return body_;
        }

    private:
        /** \brief Loads a shape from the set \a type_ and \a dimensions_, and \a resource_ if a mesh.
         *  \return A pointer to a newly allocated shape.
         */
        shapes::Shape *loadShape() const;

        /** \brief Loads a body from the loaded \a shape_.
         *  \return A pointer to a newly allocated body.
         */
        bodies::Body *loadBody() const;

        ShapeType::Type type_{ShapeType::Type::BOX};                 ///< Geometry Type.
        std::string resource_{""};                                   ///< Resource locator for MESH types.
        const Eigen::Vector3d dimensions_{Eigen::Vector3d::Ones()};  ///< Dimensions to scale geometry.
        const shapes::ShapePtr shape_{nullptr};                      ///< Loaded shape.
        const bodies::BodyPtr body_{nullptr};                        ///< Body operation.
    };

    /** \brief Collection of methods relating to transforms and transform math.
     */
    namespace TF
    {
        /** \brief Converts a vector message to an Eigen::Vector3d.
         *  \param[in] msg Message to convert.
         *  \return \a msg as an Eigen::Vector3d.
         */
        Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3 &msg);

        /** \brief Converts an Eigen::Vector3d to a vector message.
         *  \param[in] vector Vector to convert.
         *  \return \a vector as a vector message.
         */
        geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d &vector);

        /** \brief Converts a pose message to Eigen::Affine3d.
         *  \param[in] msg Message to convert.
         *  \return \a msg an Eigen::Affine3d.
         */
        Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose &msg);

        /** \brief Converts an Eigen::Affine3d to a pose message.
         *  \param[in] pose Pose to convert.
         *  \return \a pose as a pose message.
         */
        geometry_msgs::Pose poseEigenToMsg(const Eigen::Affine3d &pose);

        /** \brief Converts a quaternion message to Eigen::Quaterniond.
         *  \param[in] msg Message to convert.
         *  \return \a msg an Eigen::Quaterniond.
         */
        Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion &msg);

        /** \brief Converts an Eigen::Quaterniond to a quaternion message.
         *  \param[in] quaternion Quaternion to convert.
         *  \return \a quaternion as a quaternion message.
         */
        geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond &quaternion);

        /** \brief Get a bounding volume message for given \a geometry at a \a pose.
         *  \param[in] pose Pose to place geometry at.
         *  \param[in] geometry Geometry to get bounding volume for.
         *  \return Bounding volume message for \a geometry at \a pose.
         */
        moveit_msgs::BoundingVolume getBoundingVolume(const Eigen::Affine3d &pose, const Geometry &geometry);

        /** \brief Get a position constraint message.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] pose The pose of \a geometry in \a base_frame.
         *  \param[in] geometry The geometry describing the position constraint.
         */
        moveit_msgs::PositionConstraint getPositionConstraint(const std::string &ee_name,
                                                              const std::string &base_name,
                                                              const Eigen::Affine3d &pose,
                                                              const Geometry &geometry);

        /** \brief Get an orientation constraint message.
         *  \param[in] ee_name The name of the end-effector link.
         *  \param[in] base_name The frame of pose and orientation.
         *  \param[in] orientation The desired orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string &ee_name,
                                                                    const std::string &base_name,
                                                                    const Eigen::Quaterniond &orientation,
                                                                    const Eigen::Vector3d &tolerances);

        /** \brief Sample an orientation from a given \a orientation with XYZ Euler angle \a tolerances.
         *  \param[in] orientation The desired mean orientation.
         *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
         */
        Eigen::Quaterniond sampleOrientation(const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances);
    }  // namespace TF
}  // namespace robowflex

#endif
