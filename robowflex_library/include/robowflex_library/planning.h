#ifndef ROBOWFLEX_PLANNER_
#define ROBOWFLEX_PLANNER_

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Planner);
    class Planner
    {
    public:
        Planner(const RobotPtr &robot, const std::string &name = "")
          : robot_(robot), handler_(robot_->getHandler(), name), name_(name)
        {
        }

        Planner(Planner const &) = delete;
        void operator=(Planner const &) = delete;

        virtual planning_interface::MotionPlanResponse
        plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) = 0;

        virtual const std::vector<std::string> getPlannerConfigs() const = 0;

        const RobotPtr getRobot() const
        {
            return robot_;
        }

    protected:
        RobotPtr robot_;
        IO::Handler handler_;
        const std::string name_;
    };

    template <typename P>
    class PoolPlanner : public Planner
    {
    public:
        PoolPlanner(const RobotPtr &robot, unsigned int n = std::thread::hardware_concurrency(),
                    const std::string &name = "")
          : Planner(robot, name), pool_(n)
        {
        }

        template <typename... Args>
        bool initialize(Args &&... args)
        {
            auto planner = std::make_shared<P>(robot_, name_);
            planner->initialize(std::forward<Args>(args)...);
            planners_.emplace(std::move(planner));
        }

        planning_interface::MotionPlanResponse
        plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return !planners_.empty(); });

            auto planner = planners_.front();
            planners_.pop();

            lock.unlock();

            auto result = pool_.process([&] { return planner->plan(scene, request); });

            lock.lock();
            planners_.emplace(planner);
            cv_.notify_one();

            return result;
        }

        const std::vector<std::string> getPlannerConfigs() const override
        {
            return planners_.front()->getPlannerConfigs();
        }

    private:
        Pool<planning_interface::MotionPlanResponse> pool_;

        std::queue<PlannerPtr> planners_;
        std::mutex mutex_;
        std::condition_variable cv_;
    };

    ROBOWFLEX_CLASS_FORWARD(PipelinePlanner);
    class PipelinePlanner : public Planner
    {
    public:
        PipelinePlanner(const RobotPtr &robot, const std::string &name = "") : Planner(robot, name)
        {
        }

        PipelinePlanner(PipelinePlanner const &) = delete;
        void operator=(PipelinePlanner const &) = delete;

        planning_interface::MotionPlanResponse
        plan(const SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override;

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

        ROBOWFLEX_CLASS_FORWARD(OMPLPipelinePlanner);
        class OMPLPipelinePlanner : public PipelinePlanner
        {
        public:
            OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

            OMPLPipelinePlanner(OMPLPipelinePlanner const &) = delete;
            void operator=(OMPLPipelinePlanner const &) = delete;

            bool initialize(const std::string &config_file = "", const Settings settings = Settings(),
                            const std::string &plugin = DEFAULT_PLUGIN,
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

            const std::vector<std::string> getPlannerConfigs() const override;

        protected:
            static const std::string DEFAULT_PLUGIN;
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

        //     bool initialize(const std::string &config_file = "", const OMPL::Settings settings =
        //     Settings());

        //     planning_interface::MotionPlanResponse plan(Scene &scene,
        //                                                 const planning_interface::MotionPlanRequest
        //                                                 &request) override;

        //     const std::vector<std::string> getPlannerConfigs() const override;

        // private:
        //     ompl_interface::OMPLInterface interface_;
        //     std::vector<std::string> configs_;
        // };
    }  // namespace OMPL

    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    class MotionRequestBuilder
    {
    public:
        MotionRequestBuilder(const PlannerConstPtr &planner, const std::string &group_name);

        void setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp);
        void setStartConfiguration(const std::vector<double> &joints);
        void setGoalConfiguration(const std::vector<double> &joints);
        void setGoalRegion(const std::string &ee_name, const std::string &base_name,
                           const Eigen::Affine3d &pose, const Geometry &geometry,
                           const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);
        void addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                   const Eigen::Affine3d &pose, const Geometry &geometry,
                                   const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);
        void addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                       const Eigen::Affine3d &pose, const Geometry &geometry);
        void addPathOrientationConstraint(const std::string &ee_name, const std::string &base_name,
                                          const Eigen::Quaterniond &orientation,
                                          const Eigen::Vector3d &tolerances);
        const planning_interface::MotionPlanRequest &getRequest() const;
        moveit_msgs::Constraints &getPathConstraints();

        bool toYAMLFile(const std::string &file);
        bool fromYAMLFile(const std::string &file);
        /**
         * `requested_config` is just the planner name. If it's in the OMPL Config you passed in,
         * the planner used during motion planning will be set to that.
         */
        bool setConfig(const std::string &requested_config);

    private:
        const PlannerConstPtr planner_;
        const RobotConstPtr robot_;
        const std::string group_name_;
        const robot_model::JointModelGroup *jmg_;

        planning_interface::MotionPlanRequest request_;

        static const std::vector<std::string> DEFAULT_CONFIGS;
    };

    std::vector<double> getFinalJointPositions(planning_interface::MotionPlanResponse response);
}  // namespace robowflex

#endif
