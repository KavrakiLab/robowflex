/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_PLANNING_
#define ROBOWFLEX_DART_PLANNING_

#include <set>

#include <Eigen/Dense>

#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_dart/space.h>

namespace robowflex
{
    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(TSR)
        ROBOWFLEX_CLASS_FORWARD(TSRSet)
        ROBOWFLEX_CLASS_FORWARD(TSRConstraint)
        ROBOWFLEX_CLASS_FORWARD(World)

        ROBOWFLEX_CLASS_FORWARD(TSRGoal)
        ROBOWFLEX_CLASS_FORWARD(PlanBuilder)

        class TSRGoal : public ompl::base::GoalLazySamples
        {
        public:
            TSRGoal(const ompl::base::ProblemDefinitionPtr pdef, const ompl::base::SpaceInformationPtr &si,
                    const WorldPtr &world, const std::vector<TSRPtr> &tsrs);
            TSRGoal(const ompl::base::ProblemDefinitionPtr pdef, const ompl::base::SpaceInformationPtr &si, const WorldPtr &world, const TSRPtr tsr);

            TSRGoal(const PlanBuilder &builder, TSRPtr tsr);
            TSRGoal(const PlanBuilder &builder, const std::vector<TSRPtr> &tsrs);

            ~TSRGoal();

            bool sample(const ompl::base::GoalLazySamples *gls, ompl::base::State *state);

        private:
            StateSpace::StateType *getState(ompl::base::State *state) const;
            const StateSpace *getSpace() const;

            WorldPtr world_;
            TSRSetPtr tsr_;
            bool constrained_;
            ompl::base::StateSamplerPtr sampler_;
            ompl::base::ProblemDefinitionPtr pdef_;
            std::size_t attempts_{100};
            std::size_t maxStateCount_{100};
        };

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
            TSRGoalPtr setGoalTSR(const TSRPtr &tsr);
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
            WorldPtr world;
            std::vector<TSRPtr> constraints;
            TSRConstraintPtr constraint{nullptr};

            Eigen::VectorXd start;
            Eigen::VectorXd goal;

        protected:
            void initializeConstrained();
            void initializeUnconstrained();

            void setStateValidityChecker();
        };
    }  // namespace darts
}  // namespace robowflex
#endif
