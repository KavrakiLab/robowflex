/* Author: Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_PLANNERS_
#define ROBOWFLEX_TESSERACT_PLANNERS_

#include <robowflex_library/macros.h>

#include <tesseract_planning/basic_planner.h>
#include <tesseract_planning/ompl/chain_ompl_interface.h>

#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Planner);
    /** \endcond */

    namespace hypercube
    {
        /** \brief A function than creates a new OMPL planner. Shamelessly inspired from MoveIt's OMPL plugin.
         */
        typedef std::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &si,
                                                     const std::string &name,
                                                     const std::map<std::string, std::string> &map)>
            ConfiguredPlannerAllocator;

        class Settings
        {
        public:
            /** \brief Constructor.
             */
            Settings();

            bool simplify_solutions;               ///< Whether or not planner should simplify solutions.
            double longest_valid_segment_fraction; ///< Fraction of the C-space under which any segment is
                                                   ///< automatically considered valid.
            bool use_continuous_validator;         ///< Uses TrajOpt's continuous convex hull collision checking.
        };
        
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(OMPLChainPlanner);
        /** \endcond */

        /** \class robowflex::robow_tesseract::OMPLChainPlannerPtr
         *  \brief A shared pointer wrapper for robowflex::robow_tesseract::OMPLChainPlanner. */

        /** \class robowflex::robow_tesseract::OMPLInterfacePlannerConstPtr
         *  \brief A const shared pointer wrapper for robowflex::robow_tesseract::OMPLChainPlanner. */

        /** \brief A light wrapper over tesseract's ChainOMPLPlanner
         */
        class OMPLChainPlanner : public Planner
        {
        public:
            OMPLChainPlanner(const RobotPtr &robot, const std::string &name = "");

            bool initialize(const std::string &config_file, const Settings &settings);

            void registerPlannerAllocator(const std::string &planner_id,
                                          const ConfiguredPlannerAllocator &pa);

            /** \brief Plans using the given scene and planning request
             *  Each time this is called, a new tesseract scene is created.
             */
            planning_interface::MotionPlanResponse
            plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override;

            const std::vector<std::string> getPlannerConfigs() const override;

        protected:
            std::shared_ptr<tesseract::tesseract_planning::ChainOmplInterface> chain_interface_;
            Settings settings_;

        private:
            void registerDefaultPlanners();

            std::map<std::string, ConfiguredPlannerAllocator> known_planners_;
        };
    }  // namespace hypercube
}  // namespace robowflex

#endif
