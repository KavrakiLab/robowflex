/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_STRUCTURE_
#define ROBOWFLEX_DART_STRUCTURE_

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>
#include <robowflex_library/tf.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene)
    ROBOWFLEX_CLASS_FORWARD(Geometry)
    /** \endcond */

    namespace darts
    {
        namespace magic
        {
            static const double DEFAULT_DENSITY = 1000.;
            static const double DEFAULT_DAMPING = 0.001;
            static const double DEFAULT_RESTITUTION = 1;
        };  // namespace magic

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(ACM)
        /** \endcond */

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Structure)
        /** \endcond */

        /** \class robowflex::darts::StructurePtr
            \brief A shared pointer wrapper for robowflex::darts::Structure. */

        /** \class robowflex::darts::StructureConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::Structure. */

        /** \brief Wrapper class for a dart::dynamics::Skeleton.
         */
        class Structure
        {
        public:
            /** \name Constructors
                \{ */

            /** \brief Create an empty structure.
             *  \param[in] name Name of the structure.
             */
            Structure(const std::string &name);

            /** \brief Copy a MoveIt (robowflex::Scene) into a structure.
             *  \param[in] name Name of the structure.
             *  \param[in] scene Scene to copy.
             */
            Structure(const std::string &name, const SceneConstPtr &scene);

            /** \brief Destructor.
             */
            virtual ~Structure() = default;

            /** \brief Clones this structure with a new name.
             *  \param[in] newName Name for clone of the structure.
             *  \return The cloned structure.
             */
            StructurePtr cloneStructure(const std::string &newName) const;

            /** \} */

            /** \name Getters and Setters
                \{ */

            /** \brief Get the name of this structure.
             *  \return Name of the structure.
             */
            const std::string &getName() const;

            /** \brief Get the ACM for the structure.
             *  \return The ACM.
             */
            ACMPtr getACM();

            /** \brief Get the ACM for the structure.
             *  \return The ACM.
             */
            const ACMPtr &getACMConst() const;

            /** \brief Set the skeleton for the structure.
             *  \param[in] skeleton The new skeleton.
             */
            void setSkeleton(const dart::dynamics::SkeletonPtr &skeleton);

            /** \brief Get the underlying skeleton for the structure.
             *  \return The skeleton.
             */
            dart::dynamics::SkeletonPtr &getSkeleton();

            /** \brief Get the underlying skeleton for the structure.
             *  \return The skeleton.
             */
            const dart::dynamics::SkeletonPtr &getSkeletonConst() const;

            /** \brief Dumps the structure of the skeleton to a GraphViz file.
             *  \param[in] out Stream to output to.
             *  \param[in] standalone If false, does not include graph header.
             */
            void dumpGraphViz(std::ostream &out, bool standalone = true);

            /** \} */

            /** \name Getting and Setting Configurations
                \{ */

            /** \brief Set the value of a 1-DoF joint in the structure.
             *  \param[in] name Name of joint.
             *  \param[in] value Value to set joint.
             */
            void setJoint(const std::string &name, double value);

            /** \brief Set the value of a n-DoF joint in the structure.
             *  \param[in] name Name of joint.
             *  \param[in] value Value to set joint.
             */
            void setJoint(const std::string &name, const Eigen::Ref<const Eigen::VectorXd> &value);

            /** \brief Solve the current whole-body IK problem imposed on the structure.
             *  \return Returns true on success false on failure.
             */
            bool solveIK();

            /** \brief Set the DoF at \a index to \a value.
             *  \param[in] index Index of DoF to set.
             *  \param[in] value Value to set DoF.
             */
            void setDof(unsigned int index, double value);

            /** \} */

            /** \name Modifying Frames
                \{ */

            /** \brief Get the joint names for this structure.
             *  \return The names of all the joints in this structure.
             */
            std::vector<std::string> getJointNames() const;

            /** \brief Get a reference to the joint in the structure.
             *  \param[in] joint_name Name of the joint to retrieve.
             *  \return The joint if it exists, nullptr otherwise.
             */
            dart::dynamics::Joint *getJoint(const std::string &joint_name) const;

            /** \brief Get a body node within the structure.
             *  \param[in] name Name of the node to retrieve.
             *  \return The body node if it exists, nullptr otherwise. If name is empty, returns root.
             */
            dart::dynamics::BodyNode *getFrame(const std::string &name = "") const;

            /** \brief Get the root frame of this structure.
             *  \return The root node.
             */
            dart::dynamics::BodyNode *getRootFrame() const;

            /** \brief Reparents the child node to the parent node.
             *  \param[in] child Child to reparent.
             *  \param[in] parent Name of new parent for the child frame.
             */
            void reparentFreeFrame(dart::dynamics::BodyNode *child, const std::string &parent = "");

            /** \brief Set the transform from a joint to its parent.
             *  \param[in] name Name of joint to set transform of.
             *  \param[in] tf Transform to set.
             */
            void setJointParentTransform(const std::string &name, const RobotPose &tf);

            /** \brief Update or add a collision object.
             *  \param[in] name Name of object to add.
             *  \param[in] geometry Geometry of object.
             *  \param[in] pose Pose of object to set.
             */
            void updateCollisionObject(const std::string &name, const GeometryPtr &geometry,
                                       const robowflex::RobotPose &pose);

            /** \} */

            /** \name Constructing Frames
                \{ */

            /** \brief Add a new frame attached to a revolute joint to this structure.
             *  \param[in] properties Joint properties to use for joint.
             *  \param[in] shape Shape to attach to frame.
             *  \param[in] parent Parent frame to attach new joint to.
             *  \return The created joint and body node.
             */
            std::pair<dart::dynamics::RevoluteJoint *, dart::dynamics::BodyNode *>         //
            addRevoluteFrame(const dart::dynamics::RevoluteJoint::Properties &properties,  //
                             const dart::dynamics::ShapePtr &shape,                        //
                             dart::dynamics::BodyNode *parent = nullptr);

            /** \brief Add a new frame attached to a prismatic joint to this structure.
             *  \param[in] properties Joint properties to use for joint.
             *  \param[in] shape Shape to attach to frame.
             *  \param[in] parent Parent frame to attach new joint to.
             *  \return The created joint and body node.
             */
            std::pair<dart::dynamics::PrismaticJoint *, dart::dynamics::BodyNode *>          //
            addPrismaticFrame(const dart::dynamics::PrismaticJoint::Properties &properties,  //
                              const dart::dynamics::ShapePtr &shape,                         //
                              dart::dynamics::BodyNode *parent = nullptr);

            /** \brief Add a new frame attached to a free joint to this structure.
             *  \param[in] properties Joint properties to use for joint.
             *  \param[in] shape Shape to attach to frame.
             *  \param[in] parent Parent frame to attach new joint to.
             *  \return The created joint and body node.
             */
            std::pair<dart::dynamics::FreeJoint *, dart::dynamics::BodyNode *>     //
            addFreeFrame(const dart::dynamics::FreeJoint::Properties &properties,  //
                         const dart::dynamics::ShapePtr &shape,                    //
                         dart::dynamics::BodyNode *parent = nullptr);

            /** \brief Add a new frame attached to a fixed joint to this structure.
             *  \param[in] properties Joint properties to use for joint.
             *  \param[in] shape Shape to attach to frame.
             *  \param[in] parent Parent frame to attach new joint to.
             *  \return The created joint and body node.
             */
            std::pair<dart::dynamics::WeldJoint *, dart::dynamics::BodyNode *>       //
            addWeldedFrame(const dart::dynamics::WeldJoint::Properties &properties,  //
                           const dart::dynamics::ShapePtr &shape,                    //
                           dart::dynamics::BodyNode *parent = nullptr);

            /** \brief Add a ground plane.
             *  \param[in] z Z- height of the plane.
             *  \param[in] radius X- and Y- width of the plane.
             */
            void addGround(double z = 0., double radius = 10.);

            /** \} */

        protected:
            /** \brief Create a shape node on a body.
             *  \param[in,out] body Body to add shape node to.
             *  \param[in] shape Shape to add to body.
             */
            void createShapeNode(dart::dynamics::BodyNode *body, const dart::dynamics::ShapePtr &shape);

            const std::string name_{"robot"};                ///< Name of the structure.
            dart::dynamics::SkeletonPtr skeleton_{nullptr};  ///< Underlying skeleton.
            ACMPtr acm_;                                     ///< ACM for structure.
        };

        /** \brief Convert a robowflex::Geometry to a Dart Shape.
         *  \param[in] geometry Geometry to convert.
         *  \return Shape from geometry.
         */
        dart::dynamics::ShapePtr makeGeometry(const GeometryPtr &geometry);

        /** \brief Create a box.
         *  \param[in] v Dimensions of the box.
         *  \return The box shape.
         */
        std::shared_ptr<dart::dynamics::BoxShape> makeBox(const Eigen::Ref<const Eigen::Vector3d> &v);

        /** \brief Create a box.
         *  \param[in] x X dimension of box.
         *  \param[in] y Y dimension of box.
         *  \param[in] z Z dimension of box.
         *  \return The box shape.
         */
        std::shared_ptr<dart::dynamics::BoxShape> makeBox(double x, double y, double z);

        /** \brief Create a cylinder.
         *  \param[in] radius Radius of cylinder.
         *  \param[in] height Height of the cylinder.
         *  \return The cylinder shape.
         */
        std::shared_ptr<dart::dynamics::CylinderShape> makeCylinder(double radius, double height);

        /** \brief Create a sphere.
         *  \param[in] radius Radius of sphere.
         *  \return The sphere shape.
         */
        std::shared_ptr<dart::dynamics::SphereShape> makeSphere(double radius);

        /** \brief Create a mesh from a robowflex::Geometry that contains a mesh.
         *  \param[in] geometry Geometry with a mesh to convert.
         *  \return The mesh shape.
         */
        std::shared_ptr<dart::dynamics::MeshShape> makeMesh(const GeometryPtr &geometry);

        /** \brief Create a circle's arcsector from one angle to another, with a specified radius.
         *  \param[in] low Lower bound, in radians.
         *  \param[in] high Upper bound, in radians.
         *  \param[in] inner_radius Inner segment radius.
         *  \param[in] outer_radius Outer segment radius.
         *  \param[in] resolution Number of segments.
         *  \return The mesh shape.
         */
        std::shared_ptr<dart::dynamics::MeshShape> makeArcsegment(double low, double high,
                                                                  double inner_radius, double outer_radius,
                                                                  std::size_t resolution = 32);

        /** \brief Sets the color of the shapes on a body node.
         *  \param[in] node Node to set color of.
         *  \param[in] color Color to set.
         */
        void setColor(dart::dynamics::BodyNode *node, const Eigen::Vector4d &color);
    }  // namespace darts
}  // namespace robowflex

#endif
