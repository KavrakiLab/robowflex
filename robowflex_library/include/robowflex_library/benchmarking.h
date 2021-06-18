/* Author: Zachary Kingston, Bryce Willey */

#ifndef ROBOWFLEX_BENCHMARKING_
#define ROBOWFLEX_BENCHMARKING_

#include <cstdint>
#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <fstream>

#include <boost/variant.hpp>
#include <boost/date_time.hpp>  // for date operations

#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/planning_interface/planning_response.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/io/bag.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Trajectory);
    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    /** \endcond */

    /** \brief Variant type of possible values a run metric could be.
     */
    using PlannerMetric = boost::variant<bool, double, int>;

    std::string toMetricString(const PlannerMetric &metric);

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Profiler);
    /** \endcond */

    /** \class robowflex::ProfilerPtr
        \brief A shared pointer wrapper for robowflex::Profiler. */

    /** \class robowflex::ProfilerConstPtr
        \brief A const shared pointer wrapper for robowflex::Profiler. */

    class Profiler
    {
    public:
        /** \brief Bitmask options to select what metrics to compute for each run.
         */
        enum Metrics
        {
            WAYPOINTS = 1 << 0,   ///< Number of waypoints in path.
            CORRECT = 1 << 1,     ///< Is the path correct (no collisions?).
            LENGTH = 1 << 2,      ///< Length of the path.
            CLEARANCE = 1 << 3,   ///< Clearance of path from obstacles.
            SMOOTHNESS = 1 << 4,  ///< Smoothness of path.
        };

        /** \brief Options for profiling.
         */
        struct Options
        {
            uint32_t metrics{~0};  ///< Bitmask of which metrics to compute after planning.
            bool progress{true};   ///< If true, captures planner progress properties (if they exist).
            double progress_update_rate{0.1};  ///< Update rate for progress callbacks.
        };

        /** \brief Result of planner profiling.
         */
        struct Result
        {
            double time;                      ///< Time that planning took in seconds.
            boost::posix_time::ptime start;   ///< Query start time.
            boost::posix_time::ptime finish;  ///< Query end time.

            bool success;  ///< Was the plan successful?

            std::vector<std::string> property_names;                   ///< Planner progress value names.
            std::vector<std::map<std::string, std::string>> progress;  ///< Planner progress data.
            std::map<std::string, PlannerMetric> metrics;              ///< Map of metric name to value.

            planning_interface::MotionPlanResponse response;  ///< Planner response.
            TrajectoryPtr trajectory;                         ///< The resulting trajectory.

            /** \brief Retrieves the time series data of a planner progress property for a given X-,Y- pair of
             * progress properties. Will ignore a point if either value is non-finite.
             *  \param[in] xprop The property for the first coordinate.
             *  \param[in] yprop The property for the second coordinate.
             *  \return A vector of the points.
             */
            std::vector<std::pair<double, double>>
            getProgressPropertiesAsPoints(const std::string &xprop, const std::string &yprop) const;
        };

        /** \brief Type for callback function that returns a metric over the results of a planning query.
         *  \param[in] planner The planner being profiled.
         *  \param[in] scene The scene used for planning.
         *  \param[in] request The planning request.
         *  \param[in] run The results of the planning run, including prior computed metrics.
         *  \return The value of this metric. Will be stored in run data under associated name.
         */
        using ComputeMetricCallback =                                                          //
            std::function<PlannerMetric(const PlannerPtr &planner,                             //
                                        const SceneConstPtr &scene,                            //
                                        const planning_interface::MotionPlanRequest &request,  //
                                        const Result &run)>;

        /** \brief Allocator function that returns a planner progress property callback function for the
         * current planning request.
         *  \param[in] planner The planner being profiled.
         *  \param[in] scene The scene used for planning.
         *  \param[in] request The planning request.
         *  \return A planner progress property function.
         */
        using ProgressPropertyAllocator =                                        //
            std::function<Planner::ProgressProperty(const PlannerPtr &planner,   //
                                                    const SceneConstPtr &scene,  //
                                                    const planning_interface::MotionPlanRequest &request)>;

        /** \brief Type for callback function that is called in the planner progress property loop.
         *  \param[in] planner The planner being profiled.
         *  \param[in] scene The scene used for planning.
         *  \param[in] request The planning request.
         *  \param[in] run The results of the planning run, including prior computed metrics.
         */
        using ProgressCallback = std::function<void(const PlannerPtr &planner,                             //
                                                    const SceneConstPtr &scene,                            //
                                                    const planning_interface::MotionPlanRequest &request,  //
                                                    const Result &result)>;

        /** \brief Profiling a single plan using a \a planner.
         *  \param[in] planner Planner to profile.
         *  \param[in] scene Scene to plan in.
         *  \param[in] request Planning request to profile.
         *  \param[in] options The options for profiling.
         *  \param[out] result The results of profiling.
         *  \return True if planning succeeded, false on failure.
         */
        bool profilePlan(const PlannerPtr &planner,                             //
                         const SceneConstPtr &scene,                            //
                         const planning_interface::MotionPlanRequest &request,  //
                         const Options &options,                                //
                         Result &result);

        /** \brief Add a callback function to compute a metric at the end of planning.
         *  \param[in] name Name of the metric.
         *  \param[in] metric Function to use for callback.
         */
        void addMetricCallback(const std::string &name, const ComputeMetricCallback &metric);

        /** \brief Add a function that allocates a function that returns a planner progress property function.
         *  \param[in] name Name of the planner progress property.
         *  \param[in] allocator Allocator function.
         */
        void addProgressAllocator(const std::string &name, const ProgressPropertyAllocator &allocator);

        /** \brief Add a function that is called in the planner progress property loop.
         *  \param[in] callback Callback function to add.
         */
        void addProgressCallback(const ProgressCallback &callback);

    private:
        /** \brief Compute the built-in metrics according to the provided bitmask \a options.
         *  \param[in] options Bitmask of which built-in metrics to compute.
         *  \param[in] scene Scene used for planning and metric computation.
         *  \param[out] run Metric results.
         */
        void computeBuiltinMetrics(uint32_t options, const SceneConstPtr &scene, Result &run);

        /** \brief Compute the custom user callback metrics.
         *  \param[in] planner Planner used.
         *  \param[in] scene Scene used for planning.
         *  \param[in] request Planning request.
         *  \param[out] run Metric results.
         */
        void computeCallbackMetrics(const PlannerPtr &planner,                             //
                                    const SceneConstPtr &scene,                            //
                                    const planning_interface::MotionPlanRequest &request,  //
                                    Result &run);

        std::map<std::string, ComputeMetricCallback> callbacks_;            ///< Custom callback metrics.
        std::map<std::string, ProgressPropertyAllocator> prog_allocators_;  ///< Custom progress properties.
        std::vector<ProgressCallback> prog_callbacks_;  ///< Custom progress callback functions.
    };

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
     *
     *  For efficiency, be sure to specify in the Options which metrics you want computed. For example,
     * clearance can take a while to compute. To add more metrics, or possibly metrics that are unique to your
     * planning problems, add a custom function.
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
             *  \param[in] progress How often (times per second) should progress properties be queried.
             */
            Options(unsigned int runs = 100, uint32_t options = ~0, double progress = 0.1);

            unsigned int runs;            ///< Number of runs per query.
            uint32_t options;             ///< Bitmask of robowflex::Benchmarker::MetricOptions to compute.
            double progress_update_rate;  ///< Times per second that the planner should be queried for
                                          ///< progress properties.
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
                    const std::string operator()(int value) const;

                    /** \brief Print double metric value to string.
                     */
                    const std::string operator()(double value) const;

                    /** \brief Print boolean metric value to string.
                     */
                    const std::string operator()(bool value) const;
                };

                /** \brief Constructor.
                 *  \param[in] num Run number.
                 *  \param[in] time Time that run took.
                 *  \param[in] success Was the run successful?
                 */
                Run(int num, double time, bool success);

                int num;                            ///< Run number.
                double time;                        ///< Time that run took.
                bool success;                       ///< Was the run successful?
                moveit_msgs::RobotTrajectory path;  ///< Trajectory computed in run.

                std::vector<std::map<std::string, std::string>> progress;  ///< Planner progress data.

                std::map<std::string, MetricValue> metrics;  ///< Map of metric name to value.
            };

            /** \brief Type for callback function to add additional metrics
             */
            using ComputeMetricCallbackFn =
                std::function<void(planning_interface::MotionPlanResponse &run, Run &metrics)>;

            /** \brief Constructor.
             *  \param[in] name Name of the query.
             *  \param[in] scene The scene used for the query.
             *  \param[in] planner The planner used for the query.
             *  \param[in] builder The request builder used for the query.
             *  \param[in] options Options for the query.
             *  \param[in] fn User-defined callback function for computing run metrics.
             */
            Results(const std::string &name, const SceneConstPtr &scene, const PlannerConstPtr &planner,
                    const MotionRequestBuilderConstPtr &builder, const Options &options,
                    ComputeMetricCallbackFn fn);

            /** \brief Add a run to the set of results.
             *  \param[in] num The number of the run.
             *  \param[in] time The time the run took.
             *  \param[in] run The results of the run.
             */
            Run &addRun(int num, double time, planning_interface::MotionPlanResponse &run);

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
            ComputeMetricCallbackFn metric_callback;     ///< Callback to compute user-specified metrics
            std::vector<std::string> properties;         ///< Progress properties.

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

        /** \brief Parameters of a benchmark request.
         */
        using BenchmarkRequest = std::tuple<ScenePtr, PlannerPtr, MotionRequestBuilderPtr>;

        /** \brief Function that returns a callback for computing user-defined metrics for a given benchmark
         * request.
         */
        using MetricCallbackFnAllocator =
            std::function<Results::ComputeMetricCallbackFn(const BenchmarkRequest &)>;

        /** \brief Set the function that returns a callback function for computing user-defined metrics.
         *  \param[in] metric_alloc The allocator function.
         */
        void setMetricCallbackFnAllocator(MetricCallbackFnAllocator metric_alloc);

    private:
        /** \brief Capture planner progress.
         */
        void captureProgress(const std::map<std::string, Planner::ProgressProperty> &properties,
                             std::vector<std::map<std::string, std::string>> &progress, double rate);

        std::map<std::string, BenchmarkRequest> requests_;     ///< Requests to benchmark.
        MetricCallbackFnAllocator metric_callback_allocator_;  ///< User metric callback allocator.

        std::mutex solved_mutex_;  ///< Lock used for progress property computation.
        bool solved_;              ///< Has the current benchmarking run been solved?
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
        JSONBenchmarkOutputter(const std::string &file);

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
        TrajectoryBenchmarkOutputter(const std::string &file);

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
         *  \param[in] dumpScene If true, will output scene into log file.
         */
        OMPLBenchmarkOutputter(const std::string &prefix, bool dumpScene = true);

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
        const bool dumpScene_;      ///< If true, outputs scene information in benchmark result.
    };
}  // namespace robowflex

#endif
