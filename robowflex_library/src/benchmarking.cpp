/* Author: Zachary Kingston, Bryce Willey */

#include <boost/lexical_cast.hpp>
#include <utility>

#include <moveit/version.h>

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
        const auto &scene = std::get<0>(request.second);
        const auto &planner = std::get<1>(request.second);
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

    for (const Benchmarker::Results::Run& run : results.runs)
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
            std::string operator()(int /* dummy */) const
            {
                return "INT";
            }

            std::string operator()(double /* dummy */) const
            {
                return "REAL";
            }

            std::string operator()(bool /* dummy */) const
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
