/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_SPACE_
#define ROBOWFLEX_DART_SPACE_

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <robowflex_library/class_forward.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/joints.h>

namespace robowflex
{
    namespace darts
    {
        class StateSpace : public ompl::base::RealVectorStateSpace
        {
        public:
            friend Joint;

            ROBOWFLEX_CLASS_FORWARD(StateSampler);
            friend StateSampler;
            class StateSampler : public ompl::base::RealVectorStateSampler
            {
            public:
                StateSampler(const StateSpace *space);

                void sampleUniform(ompl::base::State *state) override;

                void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                       double distance) override;

            private:
                const std::vector<JointPtr> &joints_;
            };

            class StateType : public ompl::base::RealVectorStateSpace::StateType
            {
            public:
                StateType(unsigned int n);

                Eigen::VectorXd data;
            };

            StateSpace(WorldPtr world);
            bool isMetricSpace() const override;

            void addGroup(const std::string &name, const std::string &group, unsigned int cyclic = 0);

            void setWorldState(WorldPtr world, const ompl::base::State *state);
            void setWorldState(WorldPtr world, const Eigen::Ref<const Eigen::VectorXd> &x);
            void getWorldState(WorldPtr world, Eigen::Ref<Eigen::VectorXd> x) const;

            void enforceBounds(ompl::base::State *state) const override;

            bool satisfiesBounds(const ompl::base::State *state) const override;

            double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

            double getMaximumExtent() const override;

            bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;

            void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                             ompl::base::State *state) const override;

            ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

            ompl::base::State *allocState() const override;

            void freeState(ompl::base::State *state) const override;

            WorldPtr getWorld();
            const WorldPtr &getWorldConst() const;

            std::vector<std::size_t> getIndices() const;

        protected:
            WorldPtr world_;

            std::set<dart::dynamics::Joint *> jointset_;
            std::vector<std::size_t> indices_;

            std::vector<JointPtr> joints_;

            unsigned int numRobots_{0};
            std::vector<RobotPtr> robots_;

            ompl::RNG rng_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
