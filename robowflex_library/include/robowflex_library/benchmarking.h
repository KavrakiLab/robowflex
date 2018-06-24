/* Author: Zachary Kingston, Bryce Willey */

#ifndef ROBOWFLEX_BENCHMARKING_
#define ROBOWFLEX_BENCHMARKING_

#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Benchmarker);
    /** \endcond */

    /** \class robowflex::BenchmarkerPtr
        \brief A shared pointer wrapper for robowflex::Benchmarker. */

    /** \class robowflex::BenchmarkerConstPtr
        \brief A const shared pointer wrapper for robowflex::Benchmarker. */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(BenchmarkOutputter);
    /** \endcond */

    /** \class robowflex::BenchmarkOutputterPtr
        \brief A shared pointer wrapper for robowflex::BenchmarkOutputter. */

    /** \class robowflex::BenchmarkOutputterConstPtr
        \brief A const shared pointer wrapper for robowflex::BenchmarkOutputter. */

    /** \brief A utility to benchmark many robowflex::Planner instances against different queries, specified
     *  by robowflex::Scene and robowflex::MotionRequestBuilder instances.
     */
    class Benchmarker
    {
    public:
        /** \brief Bitmask options to select what metrics to compute for each run.
         */
        enum MetricOptions
        {
            WAYPOINTS = 1 << 0,   ///< Number of waypoints in path.
            PATH = 1 << 1,        ///< The entire path.
            CORRECT = 1 << 2,     ///< Is the path correct (no collisions?).
            LENGTH = 1 << 3,      ///< Length of the path.
            CLEARANCE = 1 << 4,   ///< Clearance of path from obstacles.
            SMOOTHNESS = 1 << 5,  ///< Smoothness of path.
        };

        /** \brief Benchmark options.
         */
        class Options
        {
        public:
            /** \brief Constructor.
             *  \param[in] runs Number of runs per query.
             *  \param[in] options Bitmask of robowflex::Benchmarker::MetricOptions to compute.
             */
            Options(unsigned int runs = 100, uint32_t options = ~0) : runs(runs), options(options)
            {
            }

            unsigned int runs;  ///< Number of runs per query.
            uint32_t options;   ///< Bitmask of robowflex::Benchmarker::MetricOptions to compute.
        };

        /** \brief Benchmark results of a single query.
         */
        class Results
        {
        public:
            /** \brief Benchmark results of a single run of a query.
             */
            class Run
            {
            public:
                /** \brief Variant type of possible values a run metric could be.
                 */
                using MetricValue = boost::variant<bool, double, int>;

                /** \brief boost::variant visitor to print metric to string.
                 */
                class toString : public boost::static_visitor<const std::string>
                {
                public:
                    /** \brief Print int metric value to string.
                     */
                    const std::string operator()(int value) const
                    {
                        return boost::lexical_cast<std::string>(boost::get<int>(value));
                    }

                    /** \brief Print double metric value to string.
                     */
                    const std::string operator()(double value) const
                    {
                        double v = boost::get<double>(value);
                        return boost::lexical_cast<std::string>(
                            (std::isfinite(v)) ? v : std::numeric_limits<double>::max());
                    }

                    /** \brief Print boolean metric value to string.
                     */
                    const std::string operator()(bool value) const
                    {
                        return boost::lexical_cast<std::string>(boost::get<bool>(value));
                    }
                };

                /** \brief Constructor.
                 *  \param[in] num Run number.
                 *  \param[in] time Time that run took.
                 *  \param[in] success Was the run successful?
                 */
                Run(int num, double time, bool success) : num(num), time(time), success(success)
                {
                }

                int num;                            ///< Run number.
                double time;                        ///< Time that run took.
                bool success;                       ///< Was the run successful?
                moveit_msgs::RobotTrajectory path;  ///< Trajectory computed in run.

                std::map<std::string, MetricValue> metrics;  ///< Map of metric name to value.
            };

            /** \brief Constructor.
             *  \param[in] name Name of the query.
             *  \param[in] scene The scene used for the query.
             *  \param[in] planner The planner used for the query.
             *  \param[in] builder The request builder used for the query.
             *  \param[in] options Options for the query.
             */
            Results(const std::string &name, const SceneConstPtr scene, const PlannerConstPtr planner,
                    const MotionRequestBuilderConstPtr builder, const Options &options)
              : name(name), scene(scene), planner(planner), builder(builder), options(options)
            {
                start = IO::getDate();
            }

            /** \brief Add a run to the set of results.
             *  \param[in] num The number of the run.
             *  \param[in] time The time the run took.
             *  \param[in] run The results of the run.
             */
            void addRun(int num, double time, planning_interface::MotionPlanResponse &run);

            /** \brief Compute a set of metrics about a \a run, storing the results in \a metrics.
             *  The only metrics computed are those specified by the bitmask in \a options.
             *  \param[in] run Run to compute metrics about.
             *  \param[out] metrics Run to store metric data in.
             */
            void computeMetric(planning_interface::MotionPlanResponse &run, Run &metrics);

            const std::string name;                      ///< Name of the query.
            const SceneConstPtr scene;                   ///< Scene used for the query.
            const PlannerConstPtr planner;               ///< Planner used for the query.
            const MotionRequestBuilderConstPtr builder;  ///< Request builder used for the query.
            const Options options;                       ///< Options for the query.

            boost::posix_time::ptime start;   ///< Query start time.
            boost::posix_time::ptime finish;  ///< Query end time (after all runs).

            std::vector<Run> runs;  ///< Run data.
        };

        /** \brief Add a request to the set of benchmarks.
         *  \param[in] name Name of the query.
         *  \param[in] scene The scene to use for the query.
         *  \param[in] planner The planner to use for the query.
         *  \param[in] request The request for \a planner in \a scene.
         */
        void addBenchmarkingRequest(const std::string &name, const ScenePtr &scene, const PlannerPtr &planner,
                                    const MotionRequestBuilderPtr &request);

        /** \brief Start benchmarking all requests given a set of \a options.
         *  \param[in] output A vector of outputter, each called at the end of each request.
         *  \param[in] options Options for benchmarking.
         */
        void benchmark(const std::vector<BenchmarkOutputterPtr> &output, const Options &options = Options());

    private:
        /** \brief Parameters of a benchmark request.
         */
        using BenchmarkRequest = std::tuple<ScenePtr, PlannerPtr, MotionRequestBuilderPtr>;

        std::map<std::string, BenchmarkRequest> requests_;  ///< Requests to benchmark.
    };

    /** \brief An abstract class for outputting benchmark results.
     */
    class BenchmarkOutputter
    {
    public:
        /** \brief Virtual destructor for cleaning up resources.
         */
        virtual ~BenchmarkOutputter() = default;

        /** \brief Write the \a results of a benchmarking query out.
         *  Must be implemented by child classes.
         *  \param[in] results The results of one query of benchmarking.
         */
        virtual void dumpResult(const Benchmarker::Results &results) = 0;
    };

    /** \brief A benchmark outputter for storing data in a single JSON file.
     */
    class JSONBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        /** \brief Constructor.
         *  \param[in] file Filename to save results to.
         */
        JSONBenchmarkOutputter(const std::string &file) : file_(file)
        {
        }

        /** \brief Destructor. Closes \a outfile_.
         */
        ~JSONBenchmarkOutputter() override;

        /** \brief Dumps \a results into \a outfile_, and opens \a outfile_ if not already done so.
         *  \param[in] results Results to dump to file.
         */
        void dumpResult(const Benchmarker::Results &results) override;

    private:
        bool is_init_{false};     ///< Have we initialized the outputter (on first result)?
        const std::string file_;  ///< Filename to open.
        std::ofstream outfile_;   ///< Output stream.
    };

    /** \brief Benchmark outputter that saves each trajectory from each run to a rosbag file.
     */
    class TrajectoryBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        /** \brief Constructor.
         *  \param[in] file Filename for rosbag.
         */
        TrajectoryBenchmarkOutputter(const std::string &file) : file_(file), bag_(file_)
        {
        }

        /** \brief Dumps all trajectories in \a results to rosbag file \a file_.
         *  The topic the trajectories are saved under is the \a name_ in \a results, or the name of the
         *  request.
         *  \param[in] results Results to dump to file.
         */
        void dumpResult(const Benchmarker::Results &results) override;

    private:
        const std::string file_;  ///< Filename.
        IO::Bag bag_;             ///< Rosbag handler.
    };

    /** \brief Benchmark outputter that saves results into OMPL benchmarking log files. If
     * `ompl_benchmark_statistics.py` is available in your PATH variable, the results are also compiled into a
     * database file.
     */
    class OMPLBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        /** \brief Constructor.
         *  \param[in] prefix Prefix to place in front of all log files generated.
         */
        OMPLBenchmarkOutputter(const std::string &prefix) : prefix_(prefix)
        {
        }

        /** \brief Destructor, runs `ompl_benchmark_statistics.py` to generate benchmarking database.
         */
        ~OMPLBenchmarkOutputter() override;

        /** \brief Dumps \a results into a OMPL benchmarking log file in \a prefix_ named after the request \a
         *  name_.
         *  \param[in] results Results to dump to file.
         */
        void dumpResult(const Benchmarker::Results &results) override;

    private:
        const std::string prefix_;  ///< Log file prefix.
    };
}  // namespace robowflex

#endif
