/* Author: Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_PLANNERS_
#define ROBOWFLEX_TESSERACT_PLANNERS_

#include <robowflex.h>
#include <tesseract_planning/basic_planner.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>

#include <moveit/ompl_interface/model_based_planning_context.h>

namespace robowflex
{
    namespace robow_tesseract
    {
        class OMPLChainPlanner : Planner
        {
        public:
            class Settings
            {
                /** \brief Constructor.
                 */ 
                Settings();

                bool simplify_solutions;          ///< Whether or not planner should simplify solutions.
                double max_solution_segment_length; ///< Maximum solution segment length.
                bool use_continuous_validator;    ///< Uses TrajOpt's continuous convex hull collision checking.
            };
            
            // TODO handle init
            bool initialize();

            planning_interface::MotionPlanResponse plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override;

        protected:
            std::shared_ptr<tesseract::tesseract_planning::ChainOmplInterface> chain_interface_;
            Settings settings_;

        private:
            void registerDefaultPlanners();

            std::map<std::string, std::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &si, const std::string& name, const std::map<std::string, std::string> config)> known_planners_;

        };
    }
}






#endif