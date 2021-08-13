/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_BENCHMARKING_
#define ROBOWFLEX_BENCHMARKING_

#include <cstdint>
#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <fstream>

#include <boost/variant.hpp>

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
    using PlannerMetric = boost::variant<bool, double, int, std::size_t, std::string>;

    /** \brief Convert a planner metric into a string.
     *  \param[in] metric The metric to convert.
     *  \return The metric as a string.
     */
    std::string toMetricString(const PlannerMetric &metric);

    /** \brief A container structure for all elements needed in a planning query, plus an identifying name.
     */
    struct PlanningQuery
    {
        /** \brief Empty constructor.
         */
        PlanningQuery() = default;

        /** \brief Constructor. Fills in fields.
         *  \param[in] name Name of this query.
         *  \param[in] scene Scene to use.
         *  \param[in] planner Planner to use to evaluate query.
         *  \param[in] request Request to give planner.
         */
        PlanningQuery(const std::string &name,     //
                      const SceneConstPtr &scene,  //
                      const PlannerPtr &planner,   //
                      const planning_interface::MotionPlanRequest &request);

        std::string name;                               ///< Name of this query.
        SceneConstPtr scene;                            ///< Scene used for the query.
        PlannerPtr planner;                             ///< Planner used for the query.
        planning_interface::MotionPlanRequest request;  ///< Request used for the query.
    };

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(PlanData);
    /** \endcond */

    /** \class robowflex::PlanDataPtr
        \brief A shared pointer wrapper for robowflex::PlanData. */

    /** \class robowflex::PlanDataConstPtr
        \brief A const shared pointer wrapper for robowflex::PlanData. */

    /** \brief Detailed statistics and metrics computed from profiling a planner's motion planning.
     */
    class PlanData
    {
    public:
        /** \name Planning Query and Response
            \{ */

        PlanningQuery query;                              ///< Query evaluated to create this data.
        planning_interface::MotionPlanResponse response;  ///< Planner response.
        bool success;                                     ///< Was the plan successful?
        TrajectoryPtr trajectory;                         ///< The resulting trajectory, if available.

        /** \} */

        /** \name Timing
            \{ */

        double time;                      ///< Time that planning took in seconds.
        boost::posix_time::ptime start;   ///< Query start time.
        boost::posix_time::ptime finish;  ///< Query end time.

        /** \} */

        /** \name Host Metadata
            \{ */

        std::string hostname;    ///< Hostname of the machine the plan was run on.
        std::size_t process_id;  ///< Process ID of the process the profiler was run in.
        std::size_t thread_id;   ///< Thread ID of profiler execution.

        /** \} */

        /** \name Metrics and Progress Properties
            \{ */

        std::vector<std::string> property_names;                   ///< Planner progress value names.
        std::vector<std::map<std::string, std::string>> progress;  ///< Planner progress data.
        std::map<std::string, PlannerMetric> metrics;              ///< Map of metric name to value.

        /** \brief Retrieves the time series data of a planner progress property for a given X-,Y- pair of
         * progress properties. Will ignore a point if either value is non-finite.
         *  \param[in] xprop The property for the first coordinate.
         *  \param[in] yprop The property for the second coordinate.
         *  \return A vector of the points.
         */
        std::vector<std::pair<double, double>> getProgressPropertiesAsPoints(const std::string &xprop,
                                                                             const std::string &yprop) const;

        /** \} */
    };

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(PlanDataSet);
    /** \endcond */

    /** \class robowflex::PlanDataSetPtr
        \brief A shared pointer wrapper for robowflex::PlanDataSet. */

    /** \class robowflex::PlanDataSetConstPtr
        \brief A const shared pointer wrapper for robowflex::PlanDataSet. */

    /** \brief Detailed statistics about a benchmark of multiple queries.
     */
    class PlanDataSet
    {
    public:
        /** \name Timing
            \{ */

        double time;                      ///< Total computation time for entire dataset.
        boost::posix_time::ptime start;   ///< Start time of dataset computation.
        boost::posix_time::ptime finish;  ///< End time for dataset computation.

        /** \} */

        /** \name Experiment Parameters
            \{ */

        double allowed_time;          ///< Allowed planning time used per query.
        std::size_t trials;           ///< Requested trials for each query.
        bool enforced_single_thread;  ///< If true, all planners were asked to run in single-threaded mode.
        bool run_till_timeout;  ///< If true, planners were run to solve the problem as many times as possible
                                ///< until time ran out.
        std::size_t threads;    ///< Threads used for dataset computation.

        /** \} */

        /** \name Query Information
            \{ */

        std::string name;                      ///< Name of this dataset.
        std::vector<std::string> query_names;  ///< All unique names used by planning queries.
        std::vector<PlanningQuery> queries;    ///< All planning queries. Note that planning queries can share
                                               ///< the same name.

        /** \} */

        /** \name Data
            \{ */

        std::map<std::string, std::vector<PlanDataPtr>> data;  ///< Map of query name to collected data.

        /** \brief Add a computed plan data under a query as a data point.
         *  \param[in] query_name Name of query to store point under.
         *  \param[in] run Run data to add to query.
         */
        void addDataPoint(const std::string &query_name, const PlanDataPtr &run);

        /** \brief Get the full data set as a flat vector.
         *  \return All plan data as a vector.
         */
        std::vector<PlanDataPtr> getFlatData() const;

        /** \} */
    };

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
            uint32_t metrics{0xffffffff};  ///< Bitmask of which metrics to compute after planning.
            bool progress{true};           ///< If true, captures planner progress properties (if they exist).
            bool progress_at_least_once{true};  ///< If true, will always run the progress loop at least once.
            double progress_update_rate{0.1};   ///< Update rate for progress callbacks.
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
                                        const PlanData &run)>;

        /** \brief Allocator function that returns a planner progress property function for the
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
                                                    const PlanData &result)>;

        /** \brief Allocator function that returns a progress property callback function for the
         * current planning request.
         *  \param[in] planner The planner being profiled.
         *  \param[in] scene The scene used for planning.
         *  \param[in] request The planning request.
         *  \return A progress callback function.
         */
        using ProgressCallbackAllocator =                               //
            std::function<ProgressCallback(const PlannerPtr &planner,   //
                                           const SceneConstPtr &scene,  //
                                           const planning_interface::MotionPlanRequest &request)>;

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
                         PlanData &result) const;

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

        /** \brief Add a function that is called in the planner progress property loop.
         *  \param[in] callback Callback function to add.
         */
        void addProgressCallbackAllocator(const ProgressCallbackAllocator &allocator);

    private:
        /** \brief Compute the built-in metrics according to the provided bitmask \a options.
         *  \param[in] options Bitmask of which built-in metrics to compute.
         *  \param[in] scene Scene used for planning and metric computation.
         *  \param[out] run Metric results.
         */
        void computeBuiltinMetrics(uint32_t options, const SceneConstPtr &scene, PlanData &run) const;

        /** \brief Compute the custom user callback metrics.
         *  \param[in] planner Planner used.
         *  \param[in] scene Scene used for planning.
         *  \param[in] request Planning request.
         *  \param[out] run Metric results.
         */
        void computeCallbackMetrics(const PlannerPtr &planner,                             //
                                    const SceneConstPtr &scene,                            //
                                    const planning_interface::MotionPlanRequest &request,  //
                                    PlanData &run) const;

        std::map<std::string, ComputeMetricCallback> callbacks_;            ///< User callback metrics.
        std::map<std::string, ProgressPropertyAllocator> prog_allocators_;  ///< User progress properties.
        std::vector<ProgressCallback> prog_callbacks_;  ///< User progress callback functions.
        std::vector<ProgressCallbackAllocator> prog_callback_allocators_;  ///< User progress callback
                                                                           ///< function allocators.
    };

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Experiment);
    /** \endcond */

    /** \class robowflex::ExperimentPtr
        \brief A shared pointer wrapper for robowflex::Experiment. */

    /** \class robowflex::ExperimentConstPtr
        \brief A const shared pointer wrapper for robowflex::Experiment. */

    /** \brief A helper class for benchmarking that controls running multiple queries*/
    class Experiment
    {
    public:
        /** \name Building Experiment
            \{ */

        /** \brief Constructor.
         *  \param[in] name Name of this experiment.
         *  \param[in] options Options for the internal profiler.
         *  \param[in] allowed_time Total planning time allowed for each query.
         *  \param[in] trials Number of trials to run each query for.
         *  \param[in] timeout If true, will re-run each query until the total time taken has exceeded the
         * allotted time.
         */
        Experiment(const std::string &name,           //
                   const Profiler::Options &options,  //
                   double allowed_time = 60.0,        //
                   std::size_t trials = 100,          //
                   bool timeout = false);

        /** \brief Add a query to the experiment for profiling.
         *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
         *  \param[in] scene Scene to use for query.
         *  \param[in] planner Planner to use for query.
         *  \param[in] request Request to use for query.
         */
        void addQuery(const std::string &planner_name,  //
                      const SceneConstPtr &scene,       //
                      const PlannerPtr &planner,        //
                      const planning_interface::MotionPlanRequest &request);

        /** \brief Add a query to the experiment for profiling.
         *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
         *  \param[in] scene Scene to use for query.
         *  \param[in] planner Planner to use for query.
         *  \param[in] request Request to use for query.
         */
        void addQuery(const std::string &planner_name,  //
                      const SceneConstPtr &scene,       //
                      const PlannerPtr &planner,        //
                      const MotionRequestBuilder &request);

        /** \brief Add a query to the experiment for profiling.
         *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
         *  \param[in] scene Scene to use for query.
         *  \param[in] planner Planner to use for query.
         *  \param[in] request Request to use for query.
         */
        void addQuery(const std::string &planner_name,  //
                      const SceneConstPtr &scene,       //
                      const PlannerPtr &planner,        //
                      const MotionRequestBuilderPtr &request);

        /** \} */

        /** \name Configuration, Getters, Setters
            \{ */

        /** \brief Get the options used by the underlying profiler.
         *  \return A reference to the options used.
         */
        Profiler::Options &getOptions();

        /** \brief Get a reference to the profiler used by the experiment. Add callback functions to this
         * profiler for custom metrics.
         *  \return The profiler.
         */
        Profiler &getProfiler();

        /** \brief Get a reference to the profiler used by the experiment.
         *  \return The profiler.
         */
        const Profiler &getProfilerConst() const;

        /** \brief Get the queries added to this experiment.
         *  \return The queries added to the experiment.
         */
        const std::vector<PlanningQuery> &getQueries() const;

        /** \brief If called, will enable planners to use multiple threads. By default, planners are requested
         * to only use one thread.
         */
        void enableMultipleRequests();

        /** \brief If called, instead of using global allotted planning time (provided in the constructor),
         * the benchmark will use the requested planning time in the planning request of the query.
         */
        void overridePlanningTime();

        /** \} */

        /** \name Callback Functions
            \{ */

        /** \brief A callback function that is called before every query is profiled.
         *  \param[in] query The query to be profiled.
         */
        using PreRunCallback = std::function<void(const PlanningQuery &query)>;

        /** \brief A callback function that is called after every query is profiled.
         *  \param[in,out] result The result computed by the profiler.
         *  \param[in] query The query that was profiled.
         */
        using PostRunCallback = std::function<void(PlanData &result, const PlanningQuery &query)>;

        /** A callback function that is called after a result is added to the dataset.
         *  \param[in,out], dataset The dataset so far.
         *  \param[in] query The query that was just profiled.
         */
        using PostQueryCallback = std::function<void(PlanDataSetPtr dataset, const PlanningQuery &query)>;

        /** \brief Set the pre-query callback function.
         *  \param[in] callback Callback to use.
         */
        void setPreRunCallback(const PreRunCallback &callback);

        /** \brief Set the post-query callback function.
         *  \param[in] callback Callback to use.
         */
        void setPostRunCallback(const PostRunCallback &callback);

        /** \brief Set the post-dataset callback function.
         *  \param[in] callback Callback to use.
         */
        void setPostQueryCallback(const PostQueryCallback &callback);

        /** \} */

        /** \brief Run benchmarking on this experiment.
         *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
         *  to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
         *  \param[in] n_threads Number of threads to use for benchmarking.
         *  \return The computed dataset.
         */
        PlanDataSetPtr benchmark(std::size_t n_threads = 1) const;

    private:
        const std::string name_;  ///< Name of this experiment.
        double allowed_time_;     ///< Allotted time to use for each query.
        std::size_t trials_;      ///< Number of trials to run each query for.
        bool timeout_;  ///< If true, will re-run planners on queries until total time taken has exceeded the
                        ///< allotted time.
        bool enforce_single_thread_{true};   ///< If true, will request each planner to only use a single
                                             ///< thread.
        bool override_planning_time_{true};  ///< If true, will override request planning time with global
                                             ///< allowed time.

        Profiler::Options options_;           ///< Options for profiler.
        Profiler profiler_;                   ///< Profiler to use for extracting data.
        std::vector<PlanningQuery> queries_;  ///< Queries to test.

        PreRunCallback pre_callback_;          ///< Pre-run callback.
        PostRunCallback post_callback_;        ///< Post-run callback.
        PostQueryCallback complete_callback_;  ///< Post-run callback with dataset.
    };

    /** \brief An abstract class for outputting benchmark results.
     */
    class PlanDataSetOutputter
    {
    public:
        /** \brief Virtual destructor for cleaning up resources.
         */
        virtual ~PlanDataSetOutputter() = default;

        /** \brief Write the \a results of a benchmarking query out.
         *  Must be implemented by child classes.
         *  \param[in] results The results of one query of benchmarking.
         */
        virtual void dump(const PlanDataSet &results) = 0;
    };

    /** \brief A benchmark outputter for storing data in a single JSON file.
     */
    class JSONPlanDataSetOutputter : public PlanDataSetOutputter
    {
    public:
        /** \brief Constructor.
         *  \param[in] file Filename to save results to.
         */
        JSONPlanDataSetOutputter(const std::string &file);

        /** \brief Destructor. Closes \a outfile_.
         */
        ~JSONPlanDataSetOutputter() override;

        /** \brief Dumps \a results into \a outfile_, and opens \a outfile_ if not already done so.
         *  \param[in] results Results to dump to file.
         */
        void dump(const PlanDataSet &results) override;

    private:
        bool is_init_{false};     ///< Have we initialized the outputter (on first result)?
        const std::string file_;  ///< Filename to open.
        std::ofstream outfile_;   ///< Output stream.
    };

    /** \brief Benchmark outputter that saves each trajectory from each run to a rosbag file.
     */
    class TrajectoryPlanDataSetOutputter : public PlanDataSetOutputter
    {
    public:
        /** \brief Constructor.
         *  \param[in] file Filename for rosbag.
         */
        TrajectoryPlanDataSetOutputter(const std::string &file);

        /** \brief Dumps all trajectories in \a results to rosbag file \a file_.
         *  The topic the trajectories are saved under is the \a name_ in \a results, or the name of the
         *  request.
         *  \param[in] results Results to dump to file.
         */
        void dump(const PlanDataSet &results) override;

    private:
        const std::string file_;  ///< Filename.
        IO::Bag bag_;             ///< Rosbag handler.
    };

    /** \brief Benchmark outputter that saves results into OMPL benchmarking log files. If
     * `ompl_benchmark_statistics.py` is available in your PATH variable, the results are also compiled into a
     * database file.
     */
    class OMPLPlanDataSetOutputter : public PlanDataSetOutputter
    {
    public:
        /** \brief Constructor.
         *  \param[in] prefix Prefix to place in front of all log files generated.
         *  \param[in] dumpScene If true, will output scene into log file.
         */
        OMPLPlanDataSetOutputter(const std::string &prefix);

        /** \brief Destructor, runs `ompl_benchmark_statistics.py` to generate benchmarking database.
         */
        ~OMPLPlanDataSetOutputter() override;

        /** \brief Dumps \a results into a OMPL benchmarking log file in \a prefix_ named after the request \a
         *  name_.
         *  \param[in] results Results to dump to file.
         */
        void dump(const PlanDataSet &results) override;

    private:
        const std::string prefix_;  ///< Log file prefix.
    };
}  // namespace robowflex

#endif
