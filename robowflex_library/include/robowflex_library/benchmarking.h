#ifndef ROBOWFLEX_BENCHMARKING_
#define ROBOWFLEX_BENCHMARKING_

namespace robowflex
{
    // Forward Declaration.
    class BenchmarkOutputter;

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
             * the request name in a bag file of the given name.
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
                Run(int num, double time, bool success) : time(time), success(success)
                {
                }

                int num;
                int waypoints;
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

        void benchmark(BenchmarkOutputter &output, const Options &options = Options());

    private:
        std::map<std::string, std::tuple<Scene &, Planner &, MotionRequestBuilder &>> requests_;
    };

    class BenchmarkOutputter
    {
    public:
        BenchmarkOutputter()
        {
        }

        virtual ~BenchmarkOutputter() = default;

        // Write one unit of output (usually a single planner) to the output.
        virtual void dumpResult(const Benchmarker::Results &results) = 0;
    };

    class JSONBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        JSONBenchmarkOutputter(const std::string &file) : BenchmarkOutputter(), file_(file)
        {
        }

        ~JSONBenchmarkOutputter() override;

        void dumpResult(const Benchmarker::Results &results) override;

    private:
        bool is_init{false};
        const std::string file_;
        std::ofstream outfile_;
    };

    class OMPLBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        OMPLBenchmarkOutputter(const std::string &prefix) : BenchmarkOutputter(), prefix_(prefix)
        {
        }

        void dumpResult(const Benchmarker::Results &results) override;

    private:
        const std::string prefix_;
    };
}  // namespace robowflex

#endif
