/* Author: Zachary Kingston */

#ifndef SE3EZ_SPACE_
#define SE3EZ_SPACE_

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace se3ez
{
    namespace constants
    {
        const double pi = dart::math::constants<double>::pi();
    }

    class StateSpace : public ompl::base::RealVectorStateSpace
    {
    public:
        SE3EZ_CLASS(Joint);
        friend Joint;

        class Joint
        {
        public:
            Joint(StateSpace *space, dart::dynamics::Joint *joint, unsigned int sizeInSpace)
              : space_(space)
              , rng_(space->rng_)
              , joint_(joint)
              , startInSpace_(space_->getDimension())
              , sizeInSpace_(sizeInSpace)
            {
                for (unsigned int i = 0; i < joint->getNumDofs(); ++i)
                {
                    auto dof = joint->getDof(i);
                    //     auto limits = dof->getPositionLimits();
                    //     double low = (limits.first < -10) ? -6.28 : limits.first;
                    //     double high = (limits.second > 10) ? 6.28 : limits.second;

                    //     addDimension(dof->getName(), low, high);
                    std::cout << dof->getName() << std::endl;
                    indices_.emplace_back(dof->getIndexInSkeleton());
                }

                // std::cout << startInSpace_ << ", " << sizeInSpace_ << std::endl;
            }

            const std::vector<std::size_t> getIndex() const
            {
                return indices_;
            }

            unsigned int getSpaceSize() const
            {
                return sizeInSpace_;
            }

            unsigned int getSpaceStart() const
            {
                return startInSpace_;
            }

            Eigen::Ref<Eigen::VectorXd> getSpaceVars(Eigen::Ref<Eigen::VectorXd> a)
            {
                return a.segment(startInSpace_, sizeInSpace_);
            }

            Eigen::Ref<const Eigen::VectorXd> getSpaceVarsConst(const Eigen::Ref<const Eigen::VectorXd> &a)
            {
                return a.segment(startInSpace_, sizeInSpace_);
            }

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

            virtual void setJoint(const Eigen::Ref<const Eigen::VectorXd> &a) const = 0;

        protected:
            StateSpace *space_;
            ompl::RNG &rng_;

            std::vector<std::size_t> indices_;
            dart::dynamics::Joint *joint_;

            unsigned int startInSpace_;
            unsigned int sizeInSpace_;
        };

        class R1Joint : public Joint
        {
        public:
            R1Joint(StateSpace *space,             //
                    dart::dynamics::Joint *joint,  //
                    double low, double high)
              : Joint(space, joint, 1), low_(low), high_(high)
            {
                space_->addDimension(low, high);
            }

            // L1
            double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                            const Eigen::Ref<const Eigen::VectorXd> &b) const override
            {
                return std::fabs(a[0] - b[0]);
            }

            double getMaximumExtent() const override
            {
                return high_ - low_;
            }

            void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                             const Eigen::Ref<const Eigen::VectorXd> &b,  //
                             double t,                                    //
                             Eigen::Ref<Eigen::VectorXd> c) const override
            {
                c[0] = a[0] + t * (b[0] - a[0]);
            }

            void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const override
            {
                double &v = a[0];
                v = (v < low_) ? low_ : ((v > high_) ? high_ : v);
            }

            bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const override
            {
                const double &v = a[0];
                return (v >= low_) and (v <= high_);
            }

            void sample(Eigen::Ref<Eigen::VectorXd> a) const override
            {
                a[0] = rng_.uniformReal(low_, high_);
            }

            void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                            const Eigen::Ref<const Eigen::VectorXd> &near,  //
                            double r) const override
            {
                a[0] = rng_.uniformReal(near[0] - r, near[0] + r);
                enforceBounds(a);
            }

            void setJoint(const Eigen::Ref<const Eigen::VectorXd> &a) const override
            {
                joint_->getSkeleton()->setPositions(indices_, a);
            }

        private:
            double low_;
            double high_;
        };

        class SO2Joint : public Joint
        {
        public:
            SO2Joint(StateSpace *space,  //
                     dart::dynamics::Joint *joint)
              : Joint(space, joint, 1)
            {
                space_->addDimension(-constants::pi, constants::pi);
            }

            // L1
            double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                            const Eigen::Ref<const Eigen::VectorXd> &b) const override
            {
                double d = std::fabs(a[0] - b[0]);
                return (d > constants::pi) ? 2.0 * constants::pi - d : d;
            }

            double getMaximumExtent() const override
            {
                return constants::pi;
            }

            void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                             const Eigen::Ref<const Eigen::VectorXd> &b,  //
                             double t,                                    //
                             Eigen::Ref<Eigen::VectorXd> c) const override
            {
                double diff = b[0] - a[0];
                if (std::fabs(diff) <= constants::pi)
                    c[0] = a[0] + diff * t;
                else
                {
                    if (diff > 0.0)
                        diff = 2.0 * constants::pi - diff;
                    else
                        diff = -2.0 * constants::pi - diff;

                    double &v = c[0];
                    v = a[0] - diff * t;

                    if (v > constants::pi)
                        v -= 2.0 * constants::pi;
                    else if (v < -constants::pi)
                        v += 2.0 * constants::pi;
                }
            }

            void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const override
            {
                double &v = a[0];
                v = std::fmod(v, 2.0 * constants::pi);
                if (v < -constants::pi)
                    v += 2.0 * constants::pi;
                else if (v >= constants::pi)
                    v -= 2.0 * constants::pi;
            }

            bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const override
            {
                const double &v = a[0];
                return (v >= -dart::math::constants<double>::pi()) and
                       (v <= dart::math::constants<double>::pi());
            }

            void sample(Eigen::Ref<Eigen::VectorXd> a) const override
            {
                a[0] = rng_.uniformReal(-dart::math::constants<double>::pi(),
                                        dart::math::constants<double>::pi());
            }

            void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                            const Eigen::Ref<const Eigen::VectorXd> &near,  //
                            double r) const override
            {
                a[0] = rng_.uniformReal(near[0] - r, near[0] + r);
                enforceBounds(a);
            }

            void setJoint(const Eigen::Ref<const Eigen::VectorXd> &a) const override
            {
                joint_->getSkeleton()->setPositions(indices_, a);
            }
        };

        class SE3Joint : public Joint
        {
        public:
            SE3Joint(StateSpace *space,  //
                     dart::dynamics::Joint *joint)
              : Joint(space, joint, 7)
            {
                auto lowW = space->world_->getWorkspaceLowConst();
                auto highW = space->world_->getWorkspaceHighConst();

                auto lowJ = joint->getPositionLowerLimits();
                auto highJ = joint->getPositionUpperLimits();

                low_ = lowW.cwiseMax(lowJ);
                high_ = highW.cwiseMin(highJ);

                space_->addDimension(low_[0], high_[0]);
                space_->addDimension(low_[1], high_[1]);
                space_->addDimension(low_[2], high_[2]);
                space_->addDimension(-1., 1.);  // q w
                space_->addDimension(-1., 1.);  // q x
                space_->addDimension(-1., 1.);  // q y
                space_->addDimension(-1., 1.);  // q z
            }

            Eigen::Quaterniond toQuat(const Eigen::Ref<const Eigen::VectorXd> &a) const
            {
                return Eigen::Quaterniond(a[3], a[4], a[5], a[6]);
            }

            // L1
            double distance(const Eigen::Ref<const Eigen::VectorXd> &a,
                            const Eigen::Ref<const Eigen::VectorXd> &b) const override
            {
                return (a.segment(0, 3) - b.segment(0, 3)).norm() + toQuat(a).angularDistance(toQuat(b));
            }

            double getMaximumExtent() const override
            {
                return (low_ - high_).norm() + 0.5 * constants::pi;
            }

            void interpolate(const Eigen::Ref<const Eigen::VectorXd> &a,  //
                             const Eigen::Ref<const Eigen::VectorXd> &b,  //
                             double t,                                    //
                             Eigen::Ref<Eigen::VectorXd> c) const override
            {
                c.segment(0, 3) = a.segment(0, 3) + t * (b.segment(0, 3) - a.segment(0, 3));
                auto slerp = toQuat(a).slerp(t, toQuat(b));
                c[3] = slerp.w();
                c[4] = slerp.x();
                c[5] = slerp.y();
                c[6] = slerp.z();
            }

            void enforceBounds(Eigen::Ref<Eigen::VectorXd> a) const override
            {
                for (unsigned int i = 0; i < 3; ++i)
                    a[i] = (a[i] < low_[i]) ? low_[i] : ((a[i] > high_[i]) ? high_[i] : a[i]);

                auto q = toQuat(a).normalized();
                a[3] = q.w();
                a[4] = q.x();
                a[5] = q.y();
                a[6] = q.z();
            }

            bool satisfiesBounds(const Eigen::Ref<const Eigen::VectorXd> &a) const override
            {
                for (unsigned int i = 0; i < 3; ++i)
                {
                    const double &v = a[i];
                    if ((v < low_[i]) or (v > high_[i]))
                        return false;
                }

                return true;
            }

            void sample(Eigen::Ref<Eigen::VectorXd> a) const override
            {
                for (unsigned int i = 0; i < 3; ++i)
                    a[i] = rng_.uniformReal(low_[i], high_[i]);

                double q[4];
                rng_.quaternion(q);
                a[3] = q[3];
                a[4] = q[0];
                a[5] = q[1];
                a[6] = q[2];
            }

            void sampleNear(Eigen::Ref<Eigen::VectorXd> a,                  //
                            const Eigen::Ref<const Eigen::VectorXd> &near,  //
                            double r) const override
            {
                for (unsigned int i = 0; i < 3; ++i)
                    a[i] = rng_.uniformReal(near[i] - r, near[i] + r);

                if (r >= .25 * constants::pi)
                {
                    double q[4];
                    rng_.quaternion(q);
                    a[3] = q[3];
                    a[4] = q[0];
                    a[5] = q[1];
                    a[6] = q[2];
                }
                else
                {
                    double d = rng_.uniform01();
                    auto qnear = toQuat(near);
                    Eigen::Quaterniond q(  //
                        Eigen::AngleAxisd(
                            2. * std::pow(d, 1. / 3.) * r,                                             //
                            Eigen::Vector3d{rng_.gaussian01(), rng_.gaussian01(), rng_.gaussian01()})  //
                    );
                    auto qs = qnear * q;
                    a[3] = qs.w();
                    a[4] = qs.x();
                    a[5] = qs.y();
                    a[6] = qs.z();
                }

                enforceBounds(a);
            }

            void setJoint(const Eigen::Ref<const Eigen::VectorXd> &a) const override
            {
                auto j = static_cast<dart::dynamics::FreeJoint *>(joint_);

                Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
                tf.translate(Eigen::Vector3d(a[0], a[1], a[2]));
                tf.rotate(toQuat(a));

                j->setRelativeTransform(tf);

                // auto euler = toQuat(a).toRotationMatrix().eulerAngles(0, 1, 2);

                // Eigen::Vector6d set;
                // set.segment(0, 3) = euler;
                // set.segment(3, 3) = a.segment(0, 3);

                // joint_->getSkeleton()->setPositions(indices_, set);
            }

        private:
            Eigen::Vector3d low_;
            Eigen::Vector3d high_;
        };

        SE3EZ_CLASS(StateSampler);
        friend StateSampler;
        class StateSampler : public ompl::base::RealVectorStateSampler
        {
        public:
            StateSampler(const StateSpace *space)
              : ompl::base::RealVectorStateSampler(space), joints_(space->joints_)
            {
            }

            void sampleUniform(ompl::base::State *state) override
            {
                auto as = state->as<StateType>();

                for (const auto &joint : joints_)
                    joint->sample(joint->getSpaceVars(as->data));
            }

            void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                   double distance) override
            {
                auto as = state->as<StateType>();
                auto an = near->as<StateType>();

                for (const auto &joint : joints_)
                    joint->sampleNear(joint->getSpaceVars(as->data), joint->getSpaceVarsConst(an->data),
                                      distance);
            }

        private:
            const std::vector<JointPtr> &joints_;
        };

        class StateType : public ompl::base::RealVectorStateSpace::StateType
        {
        public:
            StateType(unsigned int n) : data(n)
            {
                values = data.data();
            }

            Eigen::VectorXd data;
        };

        StateSpace(WorldPtr world) : ompl::base::RealVectorStateSpace(), world_(world)
        {
        }

        bool isMetricSpace() const override
        {
            return false;
        }

        void addGroup(const std::string &name, const std::string &group, unsigned int cyclic = 0)
        {
            auto robot = world_->getRobot(name);
            auto joints = robot->getGroupJoints(group);

            for (auto joint : joints)
            {
                if (jointset_.find(joint) != jointset_.end())
                {
                    // error
                    std::cerr << "doubling up" << std::endl;
                }
                else
                    jointset_.emplace(joint);

                const auto &type = joint->getType();
                if (type == "RevoluteJoint")
                {
                    auto revolute = static_cast<dart::dynamics::RevoluteJoint *>(joint);

                    if (revolute->isCyclic(0))
                    {
                        if (cyclic)
                        {
                            std::cout << "cyclic wrap!" << std::endl;

                            double low = -dart::math::constants<double>::pi() * cyclic;
                            double high = dart::math::constants<double>::pi() * cyclic;
                            auto element = std::make_shared<R1Joint>(this, joint, low, high);

                            joints_.emplace_back(element);
                        }
                        else
                        {
                            std::cout << "cycle!" << std::endl;

                            auto element = std::make_shared<SO2Joint>(this, joint);
                            joints_.emplace_back(element);
                        }
                    }
                    else
                    {
                        std::cout << "rev" << std::endl;

                        auto dof = joint->getDof(0);
                        auto limits = dof->getPositionLimits();
                        auto element = std::make_shared<R1Joint>(this, joint, limits.first, limits.second);

                        joints_.emplace_back(element);
                    }
                }

                else if (type == "PrismaticJoint")
                {
                    auto prismatic = static_cast<dart::dynamics::PrismaticJoint *>(joint);
                    std::cout << "prs" << std::endl;

                    auto dof = joint->getDof(0);
                    auto limits = dof->getPositionLimits();
                    auto element = std::make_shared<R1Joint>(this, joint, limits.first, limits.second);

                    joints_.emplace_back(element);
                }
                else if (type == "FreeJoint")
                {
                    auto prismatic = static_cast<dart::dynamics::FreeJoint *>(joint);
                    std::cout << "free" << std::endl;

                    auto element = std::make_shared<SE3Joint>(this, joint);

                    joints_.emplace_back(element);
                }
                else
                {
                    std::cout << "unsupported" << std::endl;
                }

                // for (unsigned int i = 0; i < joint->getNumDofs(); ++i)
                // {
                //     auto dof = joint->getDof(i);
                //     auto limits = dof->getPositionLimits();
                //     double low = (limits.first < -10) ? -6.28 : limits.first;
                //     double high = (limits.second > 10) ? 6.28 : limits.second;

                //     addDimension(dof->getName(), low, high);
                //     std::cout << dof->getName() << ", " << low << ", " << high << std::endl;
                //     indices.emplace_back(dof->getIndexInSkeleton());
                // }
            }
        }

        void setWorldState(const ompl::base::State *state)
        {
            const auto &as = state->as<StateType>();

            for (const auto &joint : joints_)
            {
                const auto &v = joint->getSpaceVarsConst(as->data);
                joint->setJoint(v);
            }
            // unsigned int j = 0;
            // for (unsigned int i = 0; i < numRobots_; ++i)
            // {
            //     const auto &index = indices_[i];
            //     robots_[i]->getSkeleton()->setPositions(index, as->data.segment(j, index.size()));
            //     j += index.size();
            // }
        }

        void enforceBounds(ompl::base::State *state) const override
        {
            const auto &as = state->as<StateType>();
            for (const auto &joint : joints_)
            {
                auto v = joint->getSpaceVars(as->data);
                joint->enforceBounds(v);
            }
        }

        bool satisfiesBounds(const ompl::base::State *state) const override
        {
            const auto &as = state->as<StateType>();
            for (const auto &joint : joints_)
            {
                const auto &v = joint->getSpaceVarsConst(as->data);
                if (not joint->satisfiesBounds(v))
                    return false;
            }

            return true;
        }

        double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
        {
            const auto &as1 = state1->as<StateType>();
            const auto &as2 = state2->as<StateType>();

            double d = 0;
            for (const auto &joint : joints_)
            {
                const auto &v1 = joint->getSpaceVarsConst(as1->data);
                const auto &v2 = joint->getSpaceVarsConst(as2->data);

                d += joint->distance(v1, v2);
            }

            return d;
        }

        double getMaximumExtent() const override
        {
            double d = 0;
            for (const auto &joint : joints_)
                d += joint->getMaximumExtent();

            return d;
        }

        bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override
        {
            return distance(state1, state2) <= 1e-8;
        }

        void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                         ompl::base::State *state) const override
        {
            const auto &af = from->as<StateType>();
            const auto &at = to->as<StateType>();
            auto as = state->as<StateType>();

            for (const auto &joint : joints_)
            {
                const auto &vf = joint->getSpaceVarsConst(af->data);
                const auto &vt = joint->getSpaceVarsConst(at->data);
                auto vs = joint->getSpaceVars(as->data);

                joint->interpolate(vf, vt, t, vs);
            }
        }

        ompl::base::StateSamplerPtr allocDefaultStateSampler() const override
        {
            return std::make_shared<StateSampler>(this);
        }

        ompl::base::State *allocState() const override
        {
            return new StateType(dimension_);
        }

        void freeState(ompl::base::State *state) const override
        {
            auto as = state->as<se3ez::StateSpace::StateType>();
            delete as;
        }

        WorldPtr world_;

        std::set<dart::dynamics::Joint *> jointset_;

        std::vector<JointPtr> joints_;

        unsigned int numRobots_{0};
        std::vector<RobotPtr> robots_;
        std::vector<std::vector<std::size_t>> indices_;

        ompl::RNG rng_;
    };
}  // namespace se3ez

#endif
