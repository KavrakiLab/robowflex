#ifndef ROBOWFLEX_OMPL_
#define ROBOWFLEX_OMPL_

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>

#include <robowflex_library/robowflex.h>

namespace robowflex
{
    namespace OMPL
    {
        class OMPLInterfacePlanner : public Planner
        {
        public:
            OMPLInterfacePlanner(Robot &robot);

            OMPLInterfacePlanner(OMPLInterfacePlanner const &) = delete;
            void operator=(OMPLInterfacePlanner const &) = delete;

            bool initialize(const std::string &config_file = "", const OMPL::Settings settings = Settings());

            planning_interface::MotionPlanResponse
            plan(const Scene &scene, const planning_interface::MotionPlanRequest &request) override;

            const std::vector<std::string> getPlannerConfigs() const override;

        private:
            ompl_interface::OMPLInterface interface_;
            std::vector<std::string> configs_;
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
