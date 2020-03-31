/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_PLANNING_
#define ROBOWFLEX_DART_PLANNING_

#include <Eigen/Dense>

#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_dart/space.h>

namespace robowflex
{
    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(TSR)
        ROBOWFLEX_CLASS_FORWARD(World)

        ROBOWFLEX_CLASS_FORWARD(PlanBuilder)

        class PlanBuilder
        {
        public:
            PlanBuilder(WorldPtr world);

            void addGroup(const std::string &skeleton, const std::string &name);

            void addConstraint(const TSRPtr &tsr);

            void setStartConfigurationFromWorld();
            void setStartConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q);
            void setStartConfiguration(const std::vector<double> &q);
            void sampleStartConfiguration();

            void setGoalConfigurationFromWorld();
            void setGoalConfiguration(const Eigen::Ref<const Eigen::VectorXd> &q);
            void setGoalConfiguration(const std::vector<double> &q);
            void sampleGoalConfiguration();

            StateSpace::StateType *sampleState() const;

            void initialize();
            void setup();

            StateSpace::StateType *getState(ompl::base::State *state) const;
            const StateSpace::StateType *getStateConst(const ompl::base::State *state) const;

            void animateSolutionInWorld(std::size_t times = 0) const;

            StateSpacePtr rspace{nullptr};
            ompl::base::StateSpacePtr space{nullptr};
            ompl::base::SpaceInformationPtr info{nullptr};
            ompl::geometric::SimpleSetupPtr ss{nullptr};

        protected:
            void initializeConstrained();
            void initializeUnconstrained();

            void setStateValidityChecker();

            WorldPtr world_;

            std::vector<TSRPtr> constraints_;

            Eigen::VectorXd start_;
            Eigen::VectorXd goal_;

            ompl::base::ConstraintPtr constraint_{nullptr};
        };
    }  // namespace darts
}  // namespace robowflex
#endif
