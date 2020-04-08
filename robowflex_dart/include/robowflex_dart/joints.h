/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_JOINTS_
#define ROBOWFLEX_DART_JOINTS_

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <ompl/util/RandomNumbers.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace darts
    {
        namespace constants
        {
            const double pi = dart::math::constants<double>::pi();
        }

        ROBOWFLEX_CLASS_FORWARD(World);
        ROBOWFLEX_CLASS_FORWARD(StateSpace);

        ROBOWFLEX_CLASS_FORWARD(Joint);
        class Joint
        {
        public:
            Joint(StateSpace *space, unsigned int skelIndex, unsigned int jointIndex,
                  unsigned int sizeInSpace, unsigned int startIndex, unsigned int numDof);

            Joint(StateSpace *space, dart::dynamics::Joint *joint, unsigned int sizeInSpace,
                  unsigned int startIndex = 0, unsigned int numDof = 0);

            dart::dynamics::Joint *getJoint(WorldPtr world) const;

            Eigen::Ref<Eigen::VectorXd> getSpaceVars(Eigen::Ref<Eigen::VectorXd> a);
            Eigen::Ref<const Eigen::VectorXd> getSpaceVarsConst(const Eigen::Ref<const Eigen::VectorXd> &a);
            const std::vector<std::size_t> &getIndices() const;
            std::size_t getSkeletonIndex() const;

            virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                                    const Eigen::Ref<const Eigen::VectorXd> &b) const = 0;
            virtual double getMaximumExtent() const = 0;

            virtual void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                                     const Eigen::Ref<const Eigen::VectorXd> &b,  //
                                     double t,                                    //
                                     Eigen::Ref<Eigen::VectorXd> c) const = 0;

            virtual void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const = 0;
            virtual bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const = 0;

            virtual void sample(Eigen::Ref<Eigen::VectorXd> a) const = 0;
            virtual void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                                    const Eigen::Ref<const Eigen::VectorXd> &near,  //
                                    double r) const = 0;

            virtual void setJointState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &a) const;
            virtual void getJointState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> a) const;

        protected:
            StateSpace *space_;
            ompl::RNG &rng_;

            std::vector<std::size_t> indices_;

            unsigned int skelIndex_;
            unsigned int jointIndex_;

            unsigned int startInSpace_;
            unsigned int sizeInSpace_;

            unsigned int startIndex_;
            unsigned int numDof_;
        };

        class RnJoint : public Joint
        {
        public:
            RnJoint(StateSpace *space,             //
                    dart::dynamics::Joint *joint,  //
                    double low, double high);

            RnJoint(StateSpace *space,                   //
                    dart::dynamics::Joint *joint,        //
                    unsigned int n, unsigned int start,  //
                    Eigen::VectorXd low, Eigen::VectorXd high);

            // L1
            double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                            const Eigen::Ref<const Eigen::VectorXd> &b) const override;

            double getMaximumExtent() const override;

            void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                             const Eigen::Ref<const Eigen::VectorXd> &b,  //
                             double t,                                    //
                             Eigen::Ref<Eigen::VectorXd> c) const override;

            void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const override;

            bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const override;

            void sample(Eigen::Ref<Eigen::VectorXd> a) const override;

            void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                            const Eigen::Ref<const Eigen::VectorXd> &near,  //
                            double r) const override;

        private:
            Eigen::VectorXd low_;
            Eigen::VectorXd high_;
        };

        class SO2Joint : public Joint
        {
        public:
            SO2Joint(StateSpace *space, dart::dynamics::Joint *joint);

            // L1
            double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                            const Eigen::Ref<const Eigen::VectorXd> &b) const override;

            double getMaximumExtent() const override;

            void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                             const Eigen::Ref<const Eigen::VectorXd> &b,  //
                             double t,                                    //
                             Eigen::Ref<Eigen::VectorXd> c) const override;

            void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const override;

            bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const override;

            void sample(Eigen::Ref<Eigen::VectorXd> a) const override;

            void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                            const Eigen::Ref<const Eigen::VectorXd> &near,  //
                            double r) const override;
        };

        class SO3Joint : public Joint
        {
        public:
            SO3Joint(StateSpace *space, dart::dynamics::Joint *joint);

            Eigen::Quaterniond toQuat(const Eigen::Ref<const Eigen::VectorXd> &a) const;
            void setQuat(Eigen::Ref<Eigen::VectorXd> a, const Eigen::Quaterniond &q) const;

            double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                            const Eigen::Ref<const Eigen::VectorXd> &b) const override;

            double getMaximumExtent() const override;

            void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                             const Eigen::Ref<const Eigen::VectorXd> &b,  //
                             double t,                                    //
                             Eigen::Ref<Eigen::VectorXd> c) const override;

            void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const override;

            bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const override;

            void sample(Eigen::Ref<Eigen::VectorXd> a) const override;

            void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                            const Eigen::Ref<const Eigen::VectorXd> &near,  //
                            double r) const override;

            void setJointState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &a) const override;
            void getJointState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> a) const override;

        private:
            Eigen::Vector3d low_;
            Eigen::Vector3d high_;
        };

    }  // namespace darts
}  // namespace robowflex
#endif
