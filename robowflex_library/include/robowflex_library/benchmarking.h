#ifndef ROBOWFLEX_BENCHMARKING_
#define ROBOWFLEX_BENCHMARKING_

namespace robowflex
{
    // Forward Declaration.
    ROBOWFLEX_CLASS_FORWARD(BenchmarkOutputter);

    class Benchmarker
    {
    public:
        class Options
        {
        public:
            Options() : runs(100), trajectory_output_file("")
            {
            }

            unsigned int runs;

            /**
             * If not empty, the trajectories found by each request are stored in a topic of
             * the request nae in a bag file of the given name.
             * If empty, no trajectories are output.
             */
            std::string trajectory_output_file;
        };

        class Results
        {
        public:
            class Run
            {
            public:
                Run(int num, double time, bool success) : num(num), time(time), success(success)
                {
                }

                int num;
                int waypoints;
                moveit_msgs::RobotTrajectory path;
                double time;
                /** Whether or not MoveIt returns a 'success'. */
                bool success;
                /** True if the path is actually collision free. */
                bool correct;
                double length;
                double clearance;
                double smoothness;
            };

            Results(const std::string &name, const Scene &scene, const Planner &planner,
                    const MotionRequestBuilder &builder)
              : name(name), scene(scene), planner(planner), builder(builder)
            {
                start = IO::getDate();
            }

            void addRun(int num, double time, planning_interface::MotionPlanResponse &run);
            void computeMetric(planning_interface::MotionPlanResponse &run, Run &metrics);

            const std::string name;
            const Scene &scene;
            const Planner &planner;
            const MotionRequestBuilder &builder;

            boost::posix_time::ptime start;
            boost::posix_time::ptime finish;

            std::vector<Run> runs;
        };

        Benchmarker();

        void addBenchmarkingRequest(const std::string &name, Scene &scene, Planner &planner,
                                    MotionRequestBuilder &request);

        void benchmark(const std::vector<BenchmarkOutputterPtr> &output, const Options &options = Options());

    private:
        std::map<std::string, std::tuple<Scene &, Planner &, MotionRequestBuilder &>> requests_;
    };

    class BenchmarkOutputter
    {
    public:
        virtual ~BenchmarkOutputter() = default;

        // Write one unit of output (usually a single planner) to the output.
        virtual void dumpResult(const Benchmarker::Results &results) = 0;
    };

    class JSONBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        JSONBenchmarkOutputter(const std::string &file) : file_(file)
        {
        }

        ~JSONBenchmarkOutputter() override;

        void dumpResult(const Benchmarker::Results &results) override;

    private:
        bool is_init_{false};
        const std::string file_;
        std::ofstream outfile_;
    };

    class TrajectoryBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        TrajectoryBenchmarkOutputter(const std::string &file) : file_(file), bag_(file_)
        {
        }

        void dumpResult(const Benchmarker::Results &results) override;

    private:
        /**
         * The trajectories found by each request are stored in a topic of
         * the request name in a bag file of the given name.
         */
        bool is_init_{false};
        const std::string file_;
        IO::Bag bag_;
    };

    class OMPLBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        OMPLBenchmarkOutputter(const std::string &prefix) : prefix_(prefix)
        {
        }

        void dumpResult(const Benchmarker::Results &results) override;

    private:
        const std::string prefix_;
    };
}  // namespace robowflex

#endif
