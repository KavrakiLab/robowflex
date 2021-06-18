/* Author: Zachary Kingston, Bryce Willey */

#include <boost/lexical_cast.hpp>
#include <utility>

#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
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
    };
}  // namespace

std::string robowflex::toMetricString(const PlannerMetric &metric)
{
    return boost::apply_visitor(toMetricStringVisitor(), metric);
}

///
/// PlanData
///

std::vector<std::pair<double, double>>
PlanData::getProgressPropertiesAsPoints(const std::string &xprop, const std::string &yprop) const
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
/// Profiler
///

bool Profiler::profilePlan(const PlannerPtr &planner,                             //
                           const SceneConstPtr &scene,                            //
                           const planning_interface::MotionPlanRequest &request,  //
                           const Options &options,                                //
                           PlanData &result)
{
    bool complete = false;
    std::mutex mutex;
    std::shared_ptr<std::thread> progress_thread;

    result.scene = scene;
    result.planner = planner;
    result.request = request;

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

void Profiler::computeBuiltinMetrics(uint32_t options, const SceneConstPtr &scene, PlanData &run)
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
}

void Profiler::computeCallbackMetrics(const PlannerPtr &planner,                             //
                                      const SceneConstPtr &scene,                            //
                                      const planning_interface::MotionPlanRequest &request,  //
                                      PlanData &run)
{
    for (const auto &callback : callbacks_)
        run.metrics[callback.first] = callback.second(planner, scene, request, run);
}

///
/// Benchmarker::Options
///

Benchmarker::Options::Options(unsigned int runs, uint32_t options, double progress)
  : runs(runs), options(options), progress_update_rate(progress)
{
}

///
/// Benchmarker::Results::Run::toString
///

const std::string Benchmarker::Results::Run::toString::operator()(int value) const
{
    return std::to_string(boost::get<int>(value));
}

const std::string Benchmarker::Results::Run::toString::operator()(double value) const
{
    double v = boost::get<double>(value);
    return boost::lexical_cast<std::string>((std::isfinite(v)) ? v : std::numeric_limits<double>::max());
}

const std::string Benchmarker::Results::Run::toString::operator()(bool value) const
{
    return boost::lexical_cast<std::string>(boost::get<bool>(value));
}

///
/// Benchmarker::Results::Run
///

Benchmarker::Results::Run::Run(int num, double time, bool success) : num(num), time(time), success(success)
{
}

///
/// Benchmarker::Results
///

Benchmarker::Results::Results(const std::string &name, const SceneConstPtr &scene,
                              const PlannerConstPtr &planner, const MotionRequestBuilderConstPtr &builder,
                              const Options &options, ComputeMetricCallbackFn fn)
  : name(name)
  , scene(scene)
  , planner(planner)
  , builder(builder)
  , options(options)
  , metric_callback(std::move(fn))
{
    start = IO::getDate();
}

Benchmarker::Results::Run &Benchmarker::Results::addRun(int num, double time,
                                                        planning_interface::MotionPlanResponse &run)
{
    Run metrics(num, time, run.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
    computeMetric(run, metrics);

    runs.emplace_back(metrics);
    return runs.back();
}

void Benchmarker::Results::computeMetric(planning_interface::MotionPlanResponse &run, Run &metrics)
{
    TrajectoryPtr traj;
    if (metrics.success)
        traj = std::make_shared<Trajectory>(*run.trajectory_);

    if (options.options & MetricOptions::WAYPOINTS)
        metrics.metrics["waypoints"] = metrics.success ? int(traj->getNumWaypoints()) : int(0);

    if (options.options & MetricOptions::PATH && metrics.success)
        metrics.path = traj->getMessage();

    if (options.options & MetricOptions::LENGTH)
        metrics.metrics["length"] = metrics.success ? traj->getLength() : 0.0;

    if (options.options & MetricOptions::CORRECT)
        metrics.metrics["correct"] = metrics.success ? traj->isCollisionFree(scene) : false;

    if (options.options & MetricOptions::CLEARANCE)
        metrics.metrics["clearance"] = metrics.success ? std::get<0>(traj->getClearance(scene)) : 0.0;

    if (options.options & MetricOptions::SMOOTHNESS)
        metrics.metrics["smoothness"] = metrics.success ? traj->getSmoothness() : 0.0;

    if (metric_callback)
        metric_callback(run, metrics);
}

///
/// Benchmarker
///

void Benchmarker::addBenchmarkingRequest(const std::string &name, const ScenePtr &scene,
                                         const PlannerPtr &planner, const MotionRequestBuilderPtr &request)
{
    requests_.emplace(std::piecewise_construct,     //
                      std::forward_as_tuple(name),  //
                      std::forward_as_tuple(scene, planner, request));
}

void Benchmarker::captureProgress(const std::map<std::string, Planner::ProgressProperty> &properties,
                                  std::vector<std::map<std::string, std::string>> &progress, double rate)
{
    ros::WallTime start = ros::WallTime::now();
    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(solved_mutex_);
            if (solved_)
                return;

            std::map<std::string, std::string> data;
            double time = (ros::WallTime::now() - start).toSec();

            data["time REAL"] = std::to_string(time);
            for (const auto &property : properties)
                data[property.first] = property.second();

            progress.emplace_back(data);
        }

        // Sleep until the next update
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long int>(rate * 1000)));
    }
}

void Benchmarker::benchmark(const std::vector<BenchmarkOutputterPtr> &outputs, const Options &options)
{
    unsigned int count = 0;                                      // Completed queries.
    const unsigned int total = requests_.size() * options.runs;  // Total number of benchmarking queries.
    std::shared_ptr<std::thread> thread;                         // Thread for querying progress properties.

    for (const auto &request : requests_)
    {
        // Extract all benchmarking request information
        const auto &name = request.first;
        auto &scene = std::get<0>(request.second);
        auto &planner = std::get<1>(request.second);
        const auto &builder = std::get<2>(request.second);
        const auto &msg = builder->getRequest();
        Results::ComputeMetricCallbackFn metric_callback;

        if (metric_callback_allocator_)
            metric_callback = metric_callback_allocator_(request.second);

        // Execute pre-run step.
        planner->preRun(scene, msg);

        Results results(name, scene, planner, builder, options, metric_callback);

        // Get all progress property names.
        const auto &pp = planner->getProgressProperties(scene, msg);
        if (not pp.empty())
        {
            for (const auto &property : pp)
                results.properties.emplace_back(property.first);
            results.properties.emplace_back("time REAL");
        }

        for (unsigned int j = 0; j < options.runs || options.runs == 0; ++j)
        {
            ros::WallTime start = ros::WallTime::now();
            solved_ = false;

            // Capture planner progress.
            const auto &pp = planner->getProgressProperties(scene, msg);

            std::vector<std::map<std::string, std::string>> progress;
            if (not pp.empty())
            {
                thread.reset(
                    new std::thread([&] { captureProgress(pp, progress, options.progress_update_rate); }));
            }

            // Run planning.
            planning_interface::MotionPlanResponse response = planner->plan(scene, msg);
            double time = (ros::WallTime::now() - start).toSec();

            // Notify progress thread.
            {
                std::unique_lock<std::mutex> lock(solved_mutex_);
                solved_ = true;
            }

            if (thread)
                thread->join();

            // Collate results.
            auto &run = results.addRun(j, time, response);
            run.progress = progress;

            // If runs == 0, run until time runs out.
            if (options.runs == 0)
            {
                double time_remaining = msg.allowed_planning_time - time;
                if (time_remaining < 0.)
                    break;
                builder->setAllowedPlanningTime(time_remaining);
            }

            RBX_INFO("BENCHMARKING: [ %u / %u ] Completed", ++count, total);
        }

        results.finish = IO::getDate();

        for (const BenchmarkOutputterPtr &output : outputs)
            output->dumpResult(results);
    }
}

void Benchmarker::setMetricCallbackFnAllocator(MetricCallbackFnAllocator metric_alloc)
{
    metric_callback_allocator_ = std::move(metric_alloc);
}

///
/// JSONBenchmarkOutputter
///

JSONBenchmarkOutputter::JSONBenchmarkOutputter(const std::string &file) : file_(file)
{
}

JSONBenchmarkOutputter::~JSONBenchmarkOutputter()
{
    outfile_ << "}" << std::endl;
    outfile_.close();
}

void JSONBenchmarkOutputter::dumpResult(const Benchmarker::Results &results)
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

    for (size_t i = 0; i < results.runs.size(); i++)
    {
        const Benchmarker::Results::Run &run = results.runs[i];
        outfile_ << "{";

        outfile_ << "\"name\": \"run_" << run.num << "\",";
        outfile_ << "\"time\":" << run.time << ",";
        outfile_ << "\"success\":" << run.success;

        for (const auto &metric : run.metrics)
            outfile_ << ",\"" << metric.first
                     << "\":" << boost::apply_visitor(Benchmarker::Results::Run::toString(), metric.second);

        outfile_ << "}";

        // Write the command between each run.
        if (i != results.runs.size() - 1)
            outfile_ << "," << std::endl;
    }

    outfile_ << "]";
}

///
/// TrajectoryBenchmarkOutputter
///

TrajectoryBenchmarkOutputter::TrajectoryBenchmarkOutputter(const std::string &file) : file_(file), bag_(file_)
{
}

void TrajectoryBenchmarkOutputter::dumpResult(const Benchmarker::Results &results)
{
    if (!(results.options.options & Benchmarker::MetricOptions::PATH))
    {
        RBX_WARN("These results did not save the path according to the options. Skipping.");
        return;
    }
    const std::string &name = results.name;

    for (Benchmarker::Results::Run run : results.runs)
        bag_.addMessage(name, run.path);
}

///
/// OMPLBenchmarkOutputter
///

OMPLBenchmarkOutputter::OMPLBenchmarkOutputter(const std::string &prefix, bool dumpScene)
  : prefix_(prefix), dumpScene_(dumpScene)
{
}

OMPLBenchmarkOutputter::~OMPLBenchmarkOutputter()
{
    IO::runCommand("ompl_benchmark_statistics.py " + prefix_ + "*.log");
}

void OMPLBenchmarkOutputter::dumpResult(const Benchmarker::Results &results)
{
    std::ofstream out;
    IO::createFile(out, prefix_ + results.name + ".log");

    out << "MoveIt! version " << MOVEIT_VERSION << std::endl;  // version
    out << "Experiment " << results.name << std::endl;         // experiment
    out << "Running on " << IO::getHostname() << std::endl;    // hostname
    out << "Starting at " << IO::getDate() << std::endl;       // date

    // setup
    const auto &request = results.builder->getRequestConst();

    moveit_msgs::PlanningScene scene_msg;
    results.scene->getSceneConst()->getPlanningSceneMsg(scene_msg);

    YAML::Node yaml;
    yaml["scene"] = IO::toNode(scene_msg);
    yaml["request"] = IO::toNode(request);

    YAML::Emitter yaml_out;
    yaml_out << yaml;

    out << "<<<|" << std::endl;
    if (dumpScene_)
        out << yaml_out.c_str() << std::endl;
    out << "|>>>" << std::endl;

    // random seed (fake)
    out << "0 is the random seed" << std::endl;

    // time limit
    out << request.allowed_planning_time << " seconds per run" << std::endl;

    // memory limit
    out << "-1 MB per run" << std::endl;

    // num_runs
    out << results.runs.size() << " runs per planner" << std::endl;

    // total_time

    auto duration = results.finish - results.start;
    double total = (double)duration.total_milliseconds() / 1000.;
    out << total << " seconds spent to collect the data" << std::endl;

    // num_enums / enums
    out << "0 enum types" << std::endl;

    // num_planners
    out << "1 planners" << std::endl;

    // planners_data -> planner_data
    out << request.planner_id << std::endl;  // planner_name
    out << "0 common properties" << std::endl;

    out << (results.runs[0].metrics.size() + 2) << " properties for each run" << std::endl;  // run_properties
    out << "time REAL" << std::endl;
    out << "success BOOLEAN" << std::endl;

    std::vector<std::reference_wrapper<const std::string>> keys;
    for (const auto &metric : results.runs[0].metrics)
    {
        class ToString : public boost::static_visitor<const std::string>
        {
        public:
            const std::string operator()(int /* dummy */) const
            {
                return "INT";
            }

            const std::string operator()(double /* dummy */) const
            {
                return "REAL";
            }

            const std::string operator()(bool /* dummy */) const
            {
                return "BOOLEAN";
            }
        };

        const auto &name = metric.first;
        keys.emplace_back(name);

        out << name << " " << boost::apply_visitor(ToString(), metric.second) << std::endl;
    }

    out << results.runs.size() << " runs" << std::endl;

    for (const auto &run : results.runs)
    {
        out << run.time << "; "  //
            << run.success << "; ";

        for (const auto &key : keys)
            out << boost::apply_visitor(Benchmarker::Results::Run::toString(), run.metrics.find(key)->second)
                << "; ";

        out << std::endl;
    }

    if (not results.properties.empty())
    {
        out << results.properties.size() << " progress properties for each run" << std::endl;
        for (const auto &name : results.properties)
            out << name << std::endl;

        out << results.runs.size() << " runs" << std::endl;
        for (const auto &run : results.runs)
        {
            for (const auto &point : run.progress)
            {
                for (const auto &name : results.properties)
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
    out.close();
}
