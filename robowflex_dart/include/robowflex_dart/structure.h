/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_STRUCTURE_
#define ROBOWFLEX_DART_STRUCTURE_

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Scene)

    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(ACM)
        ROBOWFLEX_CLASS_FORWARD(Structure)

        namespace magic
        {
            static const double DEFAULT_DENSITY = 1000.;
            static const double DEFAULT_DAMPING = 0.001;
        };  // namespace magic

        class Structure
        {
        public:
            struct Frame
            {
                dart::dynamics::BodyNode *node;
                dart::dynamics::Joint *joint;
            };

            Structure(const std::string &name);
            Structure(const std::string &name, const ScenePtr &scene);

            const std::string &getName() const;

            ACMPtr getACM();
            const ACMPtr &getACMConst() const;

            void setSkeleton(const dart::dynamics::SkeletonPtr &skeleton);
            dart::dynamics::SkeletonPtr &getSkeleton();
            const dart::dynamics::SkeletonPtr &getSkeletonConst() const;

            std::pair<dart::dynamics::RevoluteJoint *, dart::dynamics::BodyNode *>         //
            addRevoluteFrame(const dart::dynamics::RevoluteJoint::Properties &properties,  //
                             const dart::dynamics::ShapePtr &shape,                        //
                             dart::dynamics::BodyNode *parent = nullptr);

            std::pair<dart::dynamics::PrismaticJoint *, dart::dynamics::BodyNode *>          //
            addPrismaticFrame(const dart::dynamics::PrismaticJoint::Properties &properties,  //
                              const dart::dynamics::ShapePtr &shape,                         //
                              dart::dynamics::BodyNode *parent = nullptr);

            std::pair<dart::dynamics::FreeJoint *, dart::dynamics::BodyNode *>     //
            addFreeFrame(const dart::dynamics::FreeJoint::Properties &properties,  //
                         const dart::dynamics::ShapePtr &shape,                    //
                         dart::dynamics::BodyNode *parent = nullptr);

            std::pair<dart::dynamics::WeldJoint *, dart::dynamics::BodyNode *>       //
            addWeldedFrame(const dart::dynamics::WeldJoint::Properties &properties,  //
                           const dart::dynamics::ShapePtr &shape,                    //
                           dart::dynamics::BodyNode *parent = nullptr);

            void addGround(double z = 0.);

        protected:
            void createShapeNode(dart::dynamics::BodyNode *body, const dart::dynamics::ShapePtr &shape);

            const std::string name_{"robot"};
            dart::dynamics::SkeletonPtr skeleton_{nullptr};
            ACMPtr acm_;
        };

        dart::dynamics::ShapePtr makeBox(const Eigen::Ref<const Eigen::Vector3d> &v);
        dart::dynamics::ShapePtr makeBox(double x, double y, double z);

        void setColor(dart::dynamics::BodyNode *node, const Eigen::Vector4d &color);
    }  // namespace darts
}  // namespace robowflex

#endif
