/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_OMPL_
#define ROBOWFLEX_OMPL_

#include <moveit/ompl_interface/ompl_interface.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/planning.h>

#include <functional>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(OMPLInterfacePlanner);
        /** \endcond */

        /** \class robowflex::OMPL::OMPLInterfacePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::OMPLInterfacePlanner. */

        /** \class robowflex::OMPL::OMPLInterfacePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::OMPLInterfacePlanner. */

        /** \brief A planner that directly uses \a MoveIt!'s OMPL planning interface.
         */
        class OMPLInterfacePlanner : public Planner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot The robot to plan for.
             *  \param[in] name Optional namespace for planner.
             */
            OMPLInterfacePlanner(const RobotPtr &robot, const std::string &name = "");

            // non-copyable
            OMPLInterfacePlanner(OMPLInterfacePlanner const &) = delete;
            void operator=(OMPLInterfacePlanner const &) = delete;

            /** \brief Initialize planning pipeline.
             *  \param[in] config_file A YAML file containing OMPL planner configurations.
             *  \param[in] settings Settings to set on the parameter server.
             *  \return True upon success, false on failure.
             */
            bool initialize(const std::string &config_file = "", const OMPL::Settings settings = Settings());

            /** \brief Plan a motion given a \a request and a \a scene.
             *  Uses the planning pipeline's generatePlan() method, which goes through planning adapters.
             *  \param[in] scene A planning scene for the same \a robot_ to compute the plan in.
             *  \param[in] request The motion planning request to solve.
             *  \return The motion planning response generated by the planner.
             */
            planning_interface::MotionPlanResponse
            plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override;

            /** \brief Returns the planning context used for this motion planning request.
             *  \param[in] scene A planning scene for the same \a robot_ to compute the plan in.
             *  \param[in] request The motion planning request to solve.
             *  \return The motion planning context used by the planner.
             */
            ompl_interface::ModelBasedPlanningContextPtr getPlanningContext(
                const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) const;

            /** \brief Get the last OMPL simple setup used in planning.
             *  \return The last OMPL simple setup used.
             */
            ompl::geometric::SimpleSetupPtr getLastSimpleSetup() const;

            /** \brief Refreshes the internal planning context.
             *  \param[in] scene A planning scene for the same \a robot_ to compute the plan in.
             *  \param[in] request The motion planning request to solve.
             *  \param[in] force If true, forces a refresh of the context.
             */
            void refreshContext(const SceneConstPtr &scene,                            //
                                const planning_interface::MotionPlanRequest &request,  //
                                bool force = false) const;

            std::map<std::string, Planner::ProgressProperty>
            getProgressProperties(const SceneConstPtr &scene,
                                  const planning_interface::MotionPlanRequest &request) const override;

            std::vector<std::string> getPlannerConfigs() const override;

            void preRun(const SceneConstPtr &scene,
                        const planning_interface::MotionPlanRequest &request) override;

            /** \brief Access the OMPLInterface directly, to customize the planning process.
             */
            ompl_interface::OMPLInterface &getInterface() const;

            /**
             * \brief Type for the callback function to be called right before planning takes place, when the
             * planning context is available.
             *
             * \param[in] context The planning context, contains SimpleSetup that will be used.
             * \param[in] scene The planning scene being planned on.
             * \param[in] request The request to plan a path for.
             */
            using PrePlanCallback = std::function<void(
                const ompl_interface::ModelBasedPlanningContextPtr &context, const SceneConstPtr &scene,
                const planning_interface::MotionPlanRequest &request)>;

            /** \brief Set a callback, to be called right before a planning session for last-minute
             * configuration or external bookkeeping.
             *
             * refreshContext() will already have been called, so the SimpleSetup obtained by
             * getLastSimpleSetup() will be the one used for planning.
             */
            void setPrePlanCallback(const PrePlanCallback &prePlanCallback);

        private:
            std::unique_ptr<ompl_interface::OMPLInterface> interface_{nullptr};  ///< Planning interface.
            std::vector<std::string> configs_;                                   ///< Planning configurations.
            bool hybridize_;    ///< Whether or not planner should hybridize solutions.
            bool interpolate_;  ///< Whether or not planner should interpolate solutions.

            mutable ID::Key last_scene_id_{ID::getNullKey()};  ///< ID of last scene.
            mutable std::string last_request_hash_;            ///< Hash of last request.

            mutable ompl_interface::ModelBasedPlanningContextPtr context_;  ///< Last context.
            mutable ompl::geometric::SimpleSetupPtr ss_;  ///< Last OMPL simple setup used for
                                                          ///< planning.

            PrePlanCallback pre_plan_callback_;  ///< Callback to be called just before planning.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
