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

    /** \brief Convert a planner metric into a string.
     *  \param[in] metric The metric to convert.
     *  \return The metric as a string.
     */
    std::string toMetricString(const PlannerMetric &metric);

    struct PlanningQuery
    {
        PlanningQuery() = default;
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
        PlanningQuery query;

        double time;                      ///< Time that planning took in seconds.
        boost::posix_time::ptime start;   ///< Query start time.
        boost::posix_time::ptime finish;  ///< Query end time.

        bool success;  ///< Was the plan successful?

        std::vector<std::string> property_names;                   ///< Planner progress value names.
        std::vector<std::map<std::string, std::string>> progress;  ///< Planner progress data.
        std::map<std::string, PlannerMetric> metrics;              ///< Map of metric name to value.

        planning_interface::MotionPlanResponse response;  ///< Planner response.
        TrajectoryPtr trajectory;                         ///< The resulting trajectory.

        // Metadata about where this plan was run.
        std::string hostname;    ///< Hostname of the machine the plan was run on.
        std::size_t process_id;  ///< Process ID of the process the profiler was run in.
        std::size_t thread_id;   ///< Thread ID of profiler execution.

        /** \brief Retrieves the time series data of a planner progress property for a given X-,Y- pair of
         * progress properties. Will ignore a point if either value is non-finite.
         *  \param[in] xprop The property for the first coordinate.
         *  \param[in] yprop The property for the second coordinate.
         *  \return A vector of the points.
         */
        std::vector<std::pair<double, double>> getProgressPropertiesAsPoints(const std::string &xprop,
                                                                             const std::string &yprop) const;
    };

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(PlanDataSet);
    /** \endcond */

    /** \class robowflex::PlanDataSetPtr
        \brief A shared pointer wrapper for robowflex::PlanDataSet. */

    /** \class robowflex::PlanDataSetConstPtr
        \brief A const shared pointer wrapper for robowflex::PlanDataSet. */

    class PlanDataSet
    {
    public:
        std::string name;  ///< Name of this dataset.

        double time;                      ///< Total computation time for entire dataset.
        boost::posix_time::ptime start;   ///< Start time of dataset computation.
        boost::posix_time::ptime finish;  ///< End time for dataset computation.

        double allowed_time;          ///< Allowed time for all queries.
        bool enforced_single_thread;  ///< If true, all planners were asked to run in single-threaded mode.

        std::size_t threads;  ///< Threads used for dataset computation.

        std::map<std::string, PlanningQuery> queries;
        std::map<std::string, std::vector<PlanDataPtr>> data;

        void addDataPoint(const std::string &query_name, const PlanDataPtr &run);

        /** \brief Get the full data set as a flat vector.
         */
        std::vector<PlanDataPtr> getFlatData() const;
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
            uint32_t metrics{~0};  ///< Bitmask of which metrics to compute after planning.
            bool progress{true};   ///< If true, captures planner progress properties (if they exist).
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
    ROBOWFLEX_CLASS_FORWARD(NBenchmarker);
    /** \endcond */

    /** \class robowflex::BenchmarkerPtr
        \brief A shared pointer wrapper for robowflex::Benchmarker. */

    /** \class robowflex::BenchmarkerConstPtr
        \brief A const shared pointer wrapper for robowflex::Benchmarker. */

    // For each experiment -> set of queries (each with a name)
    class Experiment
    {
    public:
        using PreRunCallback = std::function<void(const PlanningQuery &query)>;
        using PostRunCallback = std::function<void(PlanData &result, const PlanningQuery &query)>;
        using PostQueryCallback = std::function<void(PlanDataSetPtr dataset, const PlanningQuery &query)>;

        Experiment(const std::string &name,           //
                   const Profiler::Options &options,  //
                   double allowed_time = 60.0,        //
                   std::size_t trials = 100,          //
                   bool timeout = false);

        void addQuery(const std::string &planner_name,  //
                      const SceneConstPtr &scene,       //
                      const PlannerPtr &planner,        //
                      const planning_interface::MotionPlanRequest &request);

        void addQuery(const std::string &planner_name,  //
                      const SceneConstPtr &scene,       //
                      const PlannerPtr &planner,        //
                      const MotionRequestBuilder &request);

        void addQuery(const std::string &planner_name,  //
                      const SceneConstPtr &scene,       //
                      const PlannerPtr &planner,        //
                      const MotionRequestBuilderPtr &request);

        const std::string &getName() const;
        double getAllowedTime() const;
        std::size_t getNumTrials() const;
        Profiler::Options &getOptions();
        Profiler &getProfiler();
        const Profiler &getProfilerConst() const;
        const std::map<std::string, PlanningQuery> &getQueries() const;

        void enableMultipleRequests();
        void disableMultipleRequests();

        void setPreRunCallback(const PreRunCallback &callback);
        void setPostRunCallback(const PostRunCallback &callback);
        void setPostQueryCallback(const PostQueryCallback &callback);

        PlanDataSetPtr benchmark(std::size_t n_threads = 1) const;

    private:
        const std::string name_;
        double allowed_time_;
        std::size_t trials_;
        bool timeout_;
        bool enforce_single_thread_{true};

        Profiler::Options options_;
        Profiler profiler_;
        std::map<std::string, PlanningQuery> queries_;

        PreRunCallback pre_callback_;
        PostRunCallback post_callback_;
        PostQueryCallback complete_callback_;
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
