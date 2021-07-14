/* Author: Zachary Kingston, Bryce Willey */

#include <queue>

#include <boost/lexical_cast.hpp>
#include <utility>

#include <moveit/version.h>

#include <robowflex_library/macros.h>
#include <robowflex_library/util.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>

using namespace robowflex;

///
/// PlannerMetric
///

namespace
{
    class toMetricStringVisitor : public boost::static_visitor<std::string>
    {
    public:
        std::string operator()(int value) const
        {
            return std::to_string(boost::get<int>(value));
        }

        std::string operator()(std::size_t value) const
        {
            return std::to_string(boost::get<std::size_t>(value));
        }

        std::string operator()(double value) const
        {
            double v = boost::get<double>(value);

            // [Bad Pun] No NaNs, Infs, or buts about it.
            return boost::lexical_cast<std::string>(  //
                (std::isfinite(v)) ? v : std::numeric_limits<double>::max());
        }

        std::string operator()(bool value) const
        {
            return boost::lexical_cast<std::string>(boost::get<bool>(value));
        }

        std::string operator()(std::string value) const
        {
            return boost::get<std::string>(value);
        }
    };
}  // namespace

std::string robowflex::toMetricString(const PlannerMetric &metric)
{
    return boost::apply_visitor(toMetricStringVisitor(), metric);
}

///
/// PlanningQuery
///

PlanningQuery::PlanningQuery(const std::string &name,     //
                             const SceneConstPtr &scene,  //
                             const PlannerPtr &planner,   //
                             const planning_interface::MotionPlanRequest &request)
  : name(name), scene(scene), planner(planner), request(request)
{
}

///
/// PlanData
///

std::vector<std::pair<double, double>> PlanData::getProgressPropertiesAsPoints(const std::string &xprop,
                                                                               const std::string &yprop) const
{
    std::vector<std::pair<double, double>> ret;
    for (const auto &point : progress)
    {
        auto xit = point.find(xprop);
        if (xit == point.end())
            break;

        const auto &xval = xit->second;
        double xvald = boost::lexical_cast<double>(xval);

        const auto &yit = point.find(yprop);
        if (yit == point.end())
            break;

        const auto &yval = yit->second;
        double yvald = boost::lexical_cast<double>(yval);

        if (std::isfinite(xvald) and std::isfinite(yvald))
            ret.emplace_back(xvald, yvald);
    }

    return ret;
}

///
/// PlanDataSet
///

void PlanDataSet::addDataPoint(const std::string &query_name, const PlanDataPtr &run)
{
    auto it = data.find(query_name);
    if (it == data.end())
        data.emplace(query_name, std::vector<PlanDataPtr>{run});
    else
        it->second.emplace_back(run);
}

std::vector<PlanDataPtr> PlanDataSet::getFlatData() const
{
    std::vector<PlanDataPtr> r;
    for (const auto &query : data)
        r.insert(r.end(), query.second.begin(), query.second.end());

    return r;
}

///
/// Profiler
///

bool Profiler::profilePlan(const PlannerPtr &planner,                             //
                           const SceneConstPtr &scene,                            //
                           const planning_interface::MotionPlanRequest &request,  //
                           const Options &options,                                //
                           PlanData &result) const
{
    bool complete = false;
    std::mutex mutex;
    std::shared_ptr<std::thread> progress_thread;

    result.query.scene = scene;
    result.query.planner = planner;
    result.query.request = request;

    result.start = IO::getDate();

    // Setup planner progress property thread
    std::map<std::string, Planner::ProgressProperty> prog_props;
    const auto &pp = planner->getProgressProperties(scene, request);
    prog_props.insert(pp.begin(), pp.end());

    // Add custom progress properties
    for (const auto &allocator : prog_allocators_)
        prog_props.emplace(allocator.first, allocator.second(planner, scene, request));

    // Setup progress callbacks
    std::vector<ProgressCallback> prog_call;
    prog_call.insert(prog_call.end(), prog_callbacks_.begin(), prog_callbacks_.end());

    for (const auto &allocator : prog_callback_allocators_)
        prog_call.emplace_back(allocator(planner, scene, request));

    const bool have_prog = not prog_props.empty();
    if (options.progress  //
        and (have_prog or not prog_call.empty()))
    {
        // Get names of progress properties
        if (have_prog)
        {
            for (const auto &property : prog_props)
                result.property_names.emplace_back(property.first);
            result.property_names.emplace_back("time REAL");
        }

        progress_thread.reset(new std::thread([&] {
            bool at_least_once = options.progress_at_least_once;
            while (true)
            {
                // Sleep until the next update
                IO::threadSleep(options.progress_update_rate);

                std::unique_lock<std::mutex> lock(mutex);
                if (not at_least_once and complete)
                    return;

                if (have_prog)
                {
                    std::map<std::string, std::string> data;

                    // Add time stamp
                    double time = IO::getSeconds(result.start, IO::getDate());
                    data["time REAL"] = std::to_string(time);

                    // Compute properties
                    for (const auto &property : prog_props)
                        data[property.first] = property.second();

                    result.progress.emplace_back(data);
                }

                for (const auto &callback : prog_call)
                    callback(planner, scene, request, result);

                at_least_once = false;
            }
        }));
    }

    // Plan
    result.response = planner->plan(scene, request);

    // Notify planner progress thread
    {
        std::unique_lock<std::mutex> lock(mutex);
        complete = true;
    }

    // Compute metrics and fill out results
    result.finish = IO::getDate();
    result.time = IO::getSeconds(result.start, result.finish);
    result.success = result.response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

    if (result.success)
        result.trajectory = std::make_shared<Trajectory>(*result.response.trajectory_);

    result.hostname = IO::getHostname();
    result.process_id = IO::getProcessID();
    result.thread_id = IO::getThreadID();

    computeBuiltinMetrics(options.metrics, scene, result);
    computeCallbackMetrics(planner, scene, request, result);

    if (progress_thread)
        progress_thread->join();

    return result.success;
}

void Profiler::addMetricCallback(const std::string &name, const ComputeMetricCallback &metric)
{
    callbacks_.emplace(name, metric);
}

void Profiler::addProgressAllocator(const std::string &name, const ProgressPropertyAllocator &allocator)
{
    prog_allocators_.emplace(name, allocator);
}

void Profiler::addProgressCallback(const ProgressCallback &callback)
{
    prog_callbacks_.emplace_back(callback);
}

void Profiler::addProgressCallbackAllocator(const ProgressCallbackAllocator &allocator)
{
    prog_callback_allocators_.emplace_back(allocator);
}

void Profiler::computeBuiltinMetrics(uint32_t options, const SceneConstPtr &scene, PlanData &run) const
{
    if (options & Metrics::WAYPOINTS)
        run.metrics["waypoints"] = run.success ? int(run.trajectory->getNumWaypoints()) : int(0);

    if (options & Metrics::LENGTH)
        run.metrics["length"] = run.success ? run.trajectory->getLength() : 0.0;

    if (options & Metrics::CORRECT)
        run.metrics["correct"] = run.success ? run.trajectory->isCollisionFree(scene) : false;

    if (options & Metrics::CLEARANCE)
        run.metrics["clearance"] = run.success ? std::get<0>(run.trajectory->getClearance(scene)) : 0.0;

    if (options & Metrics::SMOOTHNESS)
        run.metrics["smoothness"] = run.success ? run.trajectory->getSmoothness() : 0.0;

    run.metrics["robowflex_planner_name"] = run.query.planner->getName();
    run.metrics["robowflex_robot_name"] = run.query.planner->getRobot()->getName();

    run.metrics["request_planner_type"] = std::string(ROBOWFLEX_DEMANGLE(typeid(*run.query.planner).name()));
    run.metrics["request_planner_id"] = run.query.request.planner_id;
    run.metrics["request_group_name"] = run.query.request.group_name;
    run.metrics["request_num_planning_attempts"] = run.query.request.num_planning_attempts;

    run.metrics["machine_hostname"] = run.hostname;
    run.metrics["machine_thread_id"] = run.thread_id;
    run.metrics["machine_process_id"] = run.process_id;
}

void Profiler::computeCallbackMetrics(const PlannerPtr &planner,                             //
                                      const SceneConstPtr &scene,                            //
                                      const planning_interface::MotionPlanRequest &request,  //
                                      PlanData &run) const
{
    for (const auto &callback : callbacks_)
        run.metrics[callback.first] = callback.second(planner, scene, request, run);
}

///
/// Experiment
///

Experiment::Experiment(const std::string &name, const Profiler::Options &options,  //
                       double allowed_time, std::size_t trials, bool timeout)
  : name_(name), allowed_time_(allowed_time), trials_(trials), timeout_(timeout), options_(options)
{
}

void Experiment::addQuery(const std::string &planner_name,  //
                          const SceneConstPtr &scene,       //
                          const PlannerPtr &planner,        //
                          const planning_interface::MotionPlanRequest &request)
{
    queries_.emplace_back(planner_name, scene, planner, request);
}

void Experiment::addQuery(const std::string &planner_name,  //
                          const SceneConstPtr &scene,       //
                          const PlannerPtr &planner,        //
                          const MotionRequestBuilder &request)
{
    addQuery(planner_name, scene, planner, request.getRequestConst());
}

void Experiment::addQuery(const std::string &planner_name,  //
                          const SceneConstPtr &scene,       //
                          const PlannerPtr &planner,        //
                          const MotionRequestBuilderPtr &request)
{
    addQuery(planner_name, scene, planner, *request);
}

Profiler::Options &Experiment::getOptions()
{
    return options_;
}

Profiler &Experiment::getProfiler()
{
    return profiler_;
}

const Profiler &Experiment::getProfilerConst() const
{
    return profiler_;
}

const std::vector<PlanningQuery> &Experiment::getQueries() const
{
    return queries_;
}

void Experiment::enableMultipleRequests()
{
    enforce_single_thread_ = false;
}

void Experiment::overridePlanningTime()
{
    override_planning_time_ = false;
}

void Experiment::setPreRunCallback(const PreRunCallback &callback)
{
    pre_callback_ = callback;
}

void Experiment::setPostRunCallback(const PostRunCallback &callback)
{
    post_callback_ = callback;
}

void Experiment::setPostQueryCallback(const PostQueryCallback &callback)
{
    complete_callback_ = callback;
}

PlanDataSetPtr Experiment::benchmark(std::size_t n_threads) const
{
    // Setup dataset to return
    auto dataset = std::make_shared<PlanDataSet>();
    dataset->name = name_;
    dataset->start = IO::getDate();
    dataset->allowed_time = allowed_time_;
    dataset->trials = trials_;
    dataset->enforced_single_thread = enforce_single_thread_;
    dataset->run_till_timeout = timeout_;
    dataset->threads = n_threads;
    dataset->queries = queries_;

    struct ThreadInfo
    {
        ThreadInfo() = default;
        ThreadInfo(const PlanningQuery *query, std::size_t trial, std::size_t index)
          : query(query), trial(trial), index(index)
        {
        }

        const PlanningQuery *query;
        std::size_t trial;
        std::size_t index;
    };

    std::queue<ThreadInfo> todo;
    std::mutex mutex;

    for (std::size_t i = 0; i < queries_.size(); ++i)
    {
        const auto &query = queries_[i];

        // Check if this name is unique, if so, add it to dataset list.
        const auto &it = std::find(dataset->query_names.begin(), dataset->query_names.end(), query.name);
        if (it == dataset->query_names.end())
            dataset->query_names.emplace_back(query.name);

        for (std::size_t j = 0; j < trials_; ++j)
            todo.emplace(&query, j, i);
    }

    std::vector<std::shared_ptr<std::thread>> threads;
    std::size_t completed_queries = 0;
    std::size_t total_queries = todo.size();

    for (std::size_t i = 0; i < n_threads; ++i)
        threads.emplace_back(std::make_shared<std::thread>([&]() {
            std::size_t id = IO::getThreadID();
            while (true)
            {
                ThreadInfo info;

                {
                    std::unique_lock<std::mutex> lock(mutex);
                    if (todo.empty())  // All done, exit.
                        return;

                    info = todo.front();
                    todo.pop();
                }

                RBX_INFO("[Thread %1%] Running Query %3% `%2%` Trial [%4%/%5%]",  //
                         id, info.query->name, info.index, info.trial + 1, trials_);

                // If override, use global time. Else use query time.
                double time_remaining =
                    (override_planning_time_) ? allowed_time_ : info.query->request.allowed_planning_time;

                std::size_t timeout_trial = 0;
                while (time_remaining > 0.)
                {
                    planning_interface::MotionPlanRequest request = info.query->request;
                    request.allowed_planning_time = time_remaining;

                    if (enforce_single_thread_)
                        request.num_planning_attempts = 1;

                    // Call pre-run callbacks
                    info.query->planner->preRun(info.query->scene, request);

                    if (pre_callback_)
                        pre_callback_(*info.query);

                    // Profile query
                    auto data = std::make_shared<PlanData>();
                    profiler_.profilePlan(info.query->planner,  //
                                          info.query->scene,    //
                                          request,              //
                                          options_,             //
                                          *data);

                    // Add experiment specific metrics
                    data->metrics.emplace("query_trial", (int)info.trial);
                    data->metrics.emplace("query_index", (int)info.index);
                    data->metrics.emplace("query_timeout_trial", (int)timeout_trial);
                    data->metrics.emplace("query_start_time", IO::getSeconds(dataset->start, data->start));
                    data->metrics.emplace("query_finish_time", IO::getSeconds(dataset->start, data->finish));

                    data->query.name = log::format("%1%:%2%:%3%", info.query->name, info.trial, info.index);

                    if (timeout_)
                        data->query.name = data->query.name + log::format(":%4%", timeout_trial);

                    if (post_callback_)
                        post_callback_(*data, *info.query);

                    {
                        std::unique_lock<std::mutex> lock(mutex);

                        dataset->addDataPoint(info.query->name, data);
                        completed_queries++;

                        if (complete_callback_)
                            complete_callback_(dataset, *info.query);
                    }

                    if (timeout_)
                    {
                        time_remaining -= data->time;
                        RBX_INFO(                                                                           //
                            "[Thread %1%] Running Query %3% `%2%` till timeout, %4% seconds remaining...",  //
                            id, info.query->name, info.index, time_remaining);
                        timeout_trial++;
                    }
                    else
                        time_remaining = 0;
                }

                RBX_INFO("[Thread %1%] Completed Query %3% `%2%` Trial [%4%/%5%] Total: [%6%/%7%]",  //
                         id, info.query->name, info.index,                                           //
                         info.trial + 1, trials_,                                                    //
                         completed_queries, total_queries);
            }
        }));

    for (const auto &thread : threads)
        thread->join();

    dataset->finish = IO::getDate();
    dataset->time = IO::getSeconds(dataset->start, dataset->finish);

    return dataset;
}

///
/// JSONPlanDataSetOutputter
///

JSONPlanDataSetOutputter::JSONPlanDataSetOutputter(const std::string &file) : file_(file)
{
}

JSONPlanDataSetOutputter::~JSONPlanDataSetOutputter()
{
    outfile_ << "}" << std::endl;
    outfile_.close();
}

void JSONPlanDataSetOutputter::dump(const PlanDataSet &results)
{
    if (not is_init_)
    {
        IO::createFile(outfile_, file_);
        outfile_ << "{";
        // TODO: output specific information about the scene and planner structs?

        is_init_ = true;
    }
    else
        outfile_ << ",";

    outfile_ << "\"" << results.name << "\":[";

    const auto &data = results.getFlatData();

    for (size_t i = 0; i < data.size(); i++)
    {
        const auto &run = data[i];
        outfile_ << "{";

        outfile_ << "\"name\": \"run_" << run->query.name << "\",";
        outfile_ << "\"time\":" << run->time << ",";
        outfile_ << "\"success\":" << run->success;

        for (const auto &metric : run->metrics)
            outfile_ << ",\"" << metric.first << "\":" << toMetricString(metric.second);

        outfile_ << "}";

        // Write the command between each run.
        if (i != data.size() - 1)
            outfile_ << "," << std::endl;
    }

    outfile_ << "]";
}

///
/// TrajectoryPlanDataSetOutputter
///

TrajectoryPlanDataSetOutputter::TrajectoryPlanDataSetOutputter(const std::string &file)
  : file_(file), bag_(file_)
{
}

void TrajectoryPlanDataSetOutputter::dump(const PlanDataSet &results)
{
    const std::string &name = results.name;

    for (const auto &data : results.getFlatData())
        if (data->trajectory)
            bag_.addMessage(name, data->trajectory->getMessage());
}

///
/// OMPLPlanDataSetOutputter
///

OMPLPlanDataSetOutputter::OMPLPlanDataSetOutputter(const std::string &prefix) : prefix_(prefix)
{
}

OMPLPlanDataSetOutputter::~OMPLPlanDataSetOutputter()
{
}

void OMPLPlanDataSetOutputter::dump(const PlanDataSet &results)
{
    std::ofstream out;
    IO::createFile(out, log::format("%1%_%2%.log", prefix_, results.name));

    out << "MoveIt! version " << MOVEIT_VERSION << std::endl;  // version
    out << "Experiment " << results.name << std::endl;         // experiment
    out << "Running on " << IO::getHostname() << std::endl;    // hostname
    out << "Starting at " << results.start << std::endl;       // date

    out << "<<<|" << std::endl;
    out << "|>>>" << std::endl;

    // random seed (fake)
    out << "0 is the random seed" << std::endl;

    // time limit
    out << results.allowed_time << " seconds per run" << std::endl;

    // memory limit
    out << "-1 MB per run" << std::endl;

    // num_runs
    // out << results.data.size() << " runs per planner" << std::endl;

    // total_time
    out << results.time << " seconds spent to collect the data" << std::endl;

    // num_enums / enums
    out << "0 enum types" << std::endl;

    // num_planners
    out << results.query_names.size() << " planners" << std::endl;

    // planners_data -> planner_data
    for (const auto &name : results.query_names)
    {
        const auto &runs = results.data.find(name)->second;

        out << name << std::endl;  // planner_name
        out << "0 common properties" << std::endl;

        out << (runs[0]->metrics.size() + 2) << " properties for each run" << std::endl;  // run_properties
        out << "time REAL" << std::endl;
        out << "success BOOLEAN" << std::endl;

        std::vector<std::reference_wrapper<const std::string>> keys;
        for (const auto &metric : runs[0]->metrics)
        {
            class ToString : public boost::static_visitor<const std::string>
            {
            public:
                std::string operator()(int /* dummy */) const
                {
                    return "INT";
                }

                std::string operator()(std::size_t /* dummy */) const
                {
                    return "BIGINT";
                }

                std::string operator()(double /* dummy */) const
                {
                    return "REAL";
                }

                std::string operator()(bool /* dummy */) const
                {
                    return "BOOLEAN";
                }

                const std::string operator()(std::string /* dummy */) const
                {
                    return "VARCHAR(128)";
                }
            };

            const auto &name = metric.first;
            keys.emplace_back(name);

            out << name << " " << boost::apply_visitor(ToString(), metric.second) << std::endl;
        }

        out << runs.size() << " runs" << std::endl;

        for (const auto &run : runs)
        {
            out << run->time << "; "  //
                << run->success << "; ";

            for (const auto &key : keys)
                out << toMetricString(run->metrics.find(key)->second) << "; ";

            out << std::endl;
        }

        const auto &progress_names = runs[0]->property_names;
        if (not progress_names.empty())
        {
            out << progress_names.size() << " progress properties for each run" << std::endl;
            for (const auto &name : progress_names)
                out << name << std::endl;

            out << runs.size() << " runs" << std::endl;
            for (const auto &run : runs)
            {
                for (const auto &point : run->progress)
                {
                    for (const auto &name : progress_names)
                    {
                        auto it = point.find(name);
                        out << it->second << ",";
                    }

                    out << ";";
                }

                out << std::endl;
            }
        }

        out << "." << std::endl;
    }

    out.close();
}
