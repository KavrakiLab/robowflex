#ifndef ROBOWFLEX_PLANNER_
#define ROBOWFLEX_PLANNER_

namespace robowflex
{
    class Planner
    {
    public:
        Planner(Robot &robot) : robot_(robot), handler_(robot_.getHandler())
        {
        }

        Planner(Planner const &) = delete;
        void operator=(Planner const &) = delete;

        virtual planning_interface::MotionPlanResponse plan(const Scene &scene,
                                                            const planning_interface::MotionPlanRequest &request) = 0;

        virtual const std::vector<std::string> getPlannerConfigs() const = 0;

        const Robot &getRobot() const
        {
            return robot_;
        }

    protected:
        Robot &robot_;
        IO::Handler &handler_;
    };

    class PipelinePlanner : public Planner
    {
    public:
        PipelinePlanner(Robot &robot) : Planner(robot)
        {
        }

        PipelinePlanner(PipelinePlanner const &) = delete;
        void operator=(PipelinePlanner const &) = delete;

        planning_interface::MotionPlanResponse plan(const Scene &scene,
                                                    const planning_interface::MotionPlanRequest &request) override;

    protected:
        planning_pipeline::PlanningPipelinePtr pipeline_;
    };

    namespace OMPL
    {
        class Settings
        {
        public:
            // Initialized here so default arguments are parsed correctly in loadPipeline.
            Settings()
              : max_goal_samples(10)
              , max_goal_sampling_attempts(1000)
              , max_planning_threads(4)
              , max_solution_segment_length(0.0)
              , max_state_sampling_attempts(4)
              , minimum_waypoint_count(10)
              , simplify_solutions(true)
              , use_constraints_approximations(false)
              , display_random_valid_states(false)
              , link_for_exploration_tree("")
              , maximum_waypoint_distance(0.0)
            {
            }

            int max_goal_samples;
            int max_goal_sampling_attempts;
            int max_planning_threads;
            double max_solution_segment_length;
            int max_state_sampling_attempts;
            int minimum_waypoint_count;
            bool simplify_solutions;
            bool use_constraints_approximations;
            bool display_random_valid_states;
            std::string link_for_exploration_tree;
            double maximum_waypoint_distance;

            void setParam(IO::Handler &handler) const;
        };

        class OMPLPipelinePlanner : public PipelinePlanner
        {
        public:
            OMPLPipelinePlanner(Robot &robot);

            OMPLPipelinePlanner(OMPLPipelinePlanner const &) = delete;
            void operator=(OMPLPipelinePlanner const &) = delete;

            bool initialize(const std::string &config_file = "", const Settings settings = Settings(),
                            const std::string &plugin = "ompl_interface/OMPLPlanner",
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

            const std::vector<std::string> getPlannerConfigs() const override;

        protected:
            static const std::vector<std::string> DEFAULT_ADAPTERS;

        private:
            std::vector<std::string> configs_;
        };

        // class OMPLInterfacePlanner : public Planner
        // {
        // public:
        //     OMPLInterfacePlanner(Robot &robot);

        //     OMPLInterfacePlanner(OMPLInterfacePlanner const &) = delete;
        //     void operator=(OMPLInterfacePlanner const &) = delete;

        //     bool initialize(const std::string &config_file = "", const OMPL::Settings settings = Settings());

        //     planning_interface::MotionPlanResponse plan(Scene &scene,
        //                                                 const planning_interface::MotionPlanRequest &request)
        //                                                 override;

        //     const std::vector<std::string> getPlannerConfigs() const override;

        // private:
        //     ompl_interface::OMPLInterface interface_;
        //     std::vector<std::string> configs_;
        // };
    }  // namespace OMPL

    class MotionRequestBuilder
    {
    public:
        MotionRequestBuilder(const Planner &planner, const std::string &group_name);

        void setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp);
        void setStartConfiguration(const std::vector<double> &joints);
        void setGoalConfiguration(const std::vector<double> &joints);
        void setGoalRegion(const std::string &ee_name, const std::string &base_name, const Eigen::Affine3d &pose,
                           const Geometry &geometry, const Eigen::Quaterniond &orientation,
                           const Eigen::Vector3d &tolerances);
        void addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                   const Eigen::Affine3d &pose, const Geometry &geometry,
                                   const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);
        void addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                       const Eigen::Affine3d &pose, const Geometry &geometry);
        void addPathOrientationConstraint(const std::string &ee_name, const std::string &base_name,
                                          const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);
        const planning_interface::MotionPlanRequest &getRequest() const;

        bool toYAMLFile(const std::string &file);
        bool fromYAMLFile(const std::string &file);

    private:
        const Planner &planner_;
        const Robot &robot_;
        const std::string group_name_;
        const robot_model::JointModelGroup *jmg_;

        planning_interface::MotionPlanRequest request_;

        static const std::string DEFAULT_CONFIG;
    };
}  // namespace robowflex

#endif
