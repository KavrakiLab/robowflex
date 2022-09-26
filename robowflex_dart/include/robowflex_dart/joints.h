/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_JOINTS_
#define ROBOWFLEX_DART_JOINTS_

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlanarJoint.hpp>

#include <ompl/util/RandomNumbers.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(World);
        ROBOWFLEX_CLASS_FORWARD(StateSpace);
        /** \endcond */

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Joint);
        /** \endcond */

        /** \class robowflex::darts::JointPtr
            \brief A shared pointer wrapper for robowflex::darts::Joint. */

        /** \class robowflex::darts::JointConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::Joint. */

        /** \brief Abstract controllable joint for robowflex::darts::StateSpace.
         */
        class Joint
        {
        public:
            /** \name Constructors.
                \{ */

            /** \brief Constructor.
             *  \param[in] space State space this joint will be in.
             *  \param[in] skelIndex Index of the skeleton this joint is in.
             *  \param[in] jointIndex Index of joint in skeleton.
             *  \param[in] sizeInSpace Number of DoFs in the state space.
             *  \param[in] startIndex Starting index in a configuration vector.
             *  \param[in] numDof Number of DoFs this joint has in the skeleton.
             */
            Joint(StateSpace *space, unsigned int skelIndex, unsigned int jointIndex,
                  unsigned int sizeInSpace, unsigned int startIndex, unsigned int numDof);

            /** \brief Constructor.
             *  \param[in] space State space this joint will be in.
             *  \param[in] joint Joint in skeleton.
             *  \param[in] sizeInSpace Number of DoFs in the state space.
             *  \param[in] startIndex Starting index in a configuration vector.
             *  \param[in] numDof Number of DoFs this joint has in the skeleton.
             */
            Joint(StateSpace *space, dart::dynamics::Joint *joint, unsigned int sizeInSpace,
                  unsigned int startIndex = 0, unsigned int numDof = 0);

            /** \} */

            /** \name Configuration access.
                \{ */

            /** \brief Gets the joint configuration (subvector of variables for this joint) from a
             * configuration.
             *  \param[in] a Configuration of the full space.
             *  \return A vector of variables for this joint.
             */
            Eigen::Ref<Eigen::VectorXd> getSpaceVars(Eigen::Ref<Eigen::VectorXd> a);

            /** \brief Gets the subvector of variables for this joint from a configuration.
             *  \param[in] a Configuration of the full space.
             *  \return A vector of variables for this joint.
             */
            Eigen::Ref<const Eigen::VectorXd> getSpaceVarsConst(const Eigen::Ref<const Eigen::VectorXd> &a);

            /** \brief Get the indices in the skeleton for this joint.
             *  \return Indices of joint DoFs.
             */
            const std::vector<std::size_t> &getIndices() const;

            /** \brief Get the names of the DoF in the skeleton for this joint.
             *  \return Names of joint DoFs.
             */
            const std::vector<std::string> &getDofs() const;

            /** \brief Get the index of the skeleton for this joint in the world.
             *  \return Skeleton index.
             */
            std::size_t getSkeletonIndex() const;

            /** \brief Get the index of the joint in the skeleton.
             *  \return Joint index.
             */
            std::size_t getJointIndex() const;

            /** \brief Get the dimension of this joint.
             *  \return The dimension of the joint.
             */
            std::size_t getDimension() const;

            /** \} */

            /** \name Joint operations.
                \{ */

            /** \brief Set the upper limits of a joint.
             *  \param[in] v Upper limits of the joint.
             */
            virtual void setUpperLimits(const Eigen::Ref<const Eigen::VectorXd> &v);

            /** \brief Set the lower limits of a joint.
             *  \param[in] v Lower limits of the joint.
             */
            virtual void setLowerLimits(const Eigen::Ref<const Eigen::VectorXd> &v);

            /** \brief Get the upper limits of a joint.
                \return The upper limits of the joint.
             */
            virtual Eigen::VectorXd getUpperLimits() const;

            /** \brief Get the lower limits of a joint.
             *  \return The lower limits of the joint.
             */
            virtual Eigen::VectorXd getLowerLimits() const;

            /** \brief Compute the distance between two joint configurations.
             *  \param[in] a Joint configuration a.
             *  \param[in] b Joint configuration b.
             *  \return Distance between configurations.
             */
            virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                                    const Eigen::Ref<const Eigen::VectorXd> &b) const = 0;

            /** \brief Get the maximum extent of this joint (maximum distance).
             *  \return The maximum extent.
             */
            virtual double getMaximumExtent() const = 0;

            /** \brief Interpolate to a new configuration \a c which is \a t from \a a to \a b.
             *  \param[in] a Joint configuration a.
             *  \param[in] b Joint configuration b.
             *  \param[in] t Amount to interpolate.
             *  \param[out] c Interpolated configuration.
             */
            virtual void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                                     const Eigen::Ref<const Eigen::VectorXd> &b,  //
                                     double t,                                    //
                                     Eigen::Ref<Eigen::VectorXd> c) const = 0;

            /** \brief Enforce bounds on a joint configuration.
             *  \param[in,out] a Joint configuration to modify.
             */
            virtual void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const = 0;

            /** \brief Check if a joint configuration satisfies bounds.
             *  \param[in,out] a Joint configuration to check.
             *  \return True if configuration satisfies bounds, false otherwise.
             */
            virtual bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const = 0;

            /** \brief Sample a configuration for this joint.
             *  \param[out] a Joint configuration to sample.
             */
            virtual void sample(Eigen::Ref<Eigen::VectorXd> a) const = 0;

            /** \brief Sample a configuration for this joint near another configuration \a near.
             *  \param[out] a Joint configuration to sample.
             *  \param[in] near Joint configuration to sample near.
             *  \param[in] r Distance from near to sample within.
             */
            virtual void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                                    const Eigen::Ref<const Eigen::VectorXd> &near,  //
                                    double r) const = 0;
            /** \} */

            /** \name World modification.
                \{ */

            /** \brief Set the state of the joint in \a world.
             *  \param[out] world World to set joint state in.
             *  \param[in] a Joint configuration to set.
             */
            virtual void setJointState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &a) const;
            /** \brief Get the state of the joint in \a world.
             *  \param[in] world World to get joint state from.
             *  \param[out] a Joint configuration to set.
             */
            virtual void getJointState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> a) const;

            /** \brief Get the Dart joint from a world.
             *  \param[in] world World to get joint from.
             *  \return Joint.
             */
            dart::dynamics::Joint *getJoint(WorldPtr world) const;

            /** \} */

        protected:
            StateSpace *space_;  ///< State space this joint is for.
            ompl::RNG &rng_;     ///< Random number generator.

            std::vector<std::size_t> indices_;  ///< Indices this joint corresponds to.
            std::vector<std::string> dofs_;     ///< Controlled DoF names.

            unsigned int skelIndex_;   ///< Index of skeleton.
            unsigned int jointIndex_;  ///< Index of joint.

            unsigned int startInSpace_;  ///< Start index in space configuration.
            unsigned int sizeInSpace_;   ///< Size of joint in space configuration.

            unsigned int startIndex_;  ///< Start index in joint.
            unsigned int numDof_;      ///< Number of DoF this joint controls.
        };

        /** \brief A real vector joint of \a n dimensions.
         */
        class RnJoint : public Joint
        {
        public:
            /** \name Constructors.
                \{ */

            /** \brief Constructor.
             *  \param[in] space State space.
             *  \param[in] joint 1-DoF joint to control.
             *  \param[in] low Lower bound.
             *  \param[in] high Upper bound.
             */
            RnJoint(StateSpace *space,             //
                    dart::dynamics::Joint *joint,  //
                    double low, double high);

            /** \brief Constructor.
             *  \param[in] space State space.
             *  \param[in] joint n-DoF joint to control.
             *  \param[in] n Number of DoF.
             *  \param[in] start Which DoF to start controlling.
             *  \param[in] low Lower bound.
             *  \param[in] high Upper bound.
             */
            RnJoint(StateSpace *space,                   //
                    dart::dynamics::Joint *joint,        //
                    unsigned int n, unsigned int start,  //
                    Eigen::VectorXd low, Eigen::VectorXd high);

            /** \} */

            void setUpperLimits(const Eigen::Ref<const Eigen::VectorXd> &v) override;
            void setLowerLimits(const Eigen::Ref<const Eigen::VectorXd> &v) override;

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
            Eigen::VectorXd low_;   ///< Lower bounds.
            Eigen::VectorXd high_;  ///< Upper bounds.
        };

        /** \brief A SO(2) joint. Bounds are from -pi to pi, and wraps.
         */
        class SO2Joint : public Joint
        {
        public:
            /** \brief Constructor.
             *  \param[in] space State space.
             *  \param[in] joint Joint.
             */
            SO2Joint(StateSpace *space, dart::dynamics::Joint *joint, unsigned int index = 0);

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

        /** \brief A SO(3) joint modeled with quaternions.
         */
        class SO3Joint : public Joint
        {
        public:
            /** \brief Constructor.
             *  \param[in] space State space.
             *  \param[in] joint Joint.
             */
            SO3Joint(StateSpace *space, dart::dynamics::Joint *joint);

            /** \brief Get the quaternion corresponding to the joint configuration.
             *  \param[in] a Joint configuration.
             *  \return Quaternion of configuration.
             */
            Eigen::Quaterniond toQuat(const Eigen::Ref<const Eigen::VectorXd> &a) const;

            /** \brief Set the joint configuration to a quaternion.
             *  \param[out] a Joint configuration.
             *  \param[in] q Quaternion to set.
             */
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
        };

    }  // namespace darts
}  // namespace robowflex
#endif
