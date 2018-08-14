/* Author: Zachary Kingston, Bryce Willey */

#include <boost/lexical_cast.hpp>

#include <moveit/version.h>
#include <moveit/collision_detection/collision_common.h>

#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/path.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/benchmarking.h>

using namespace robowflex;

///
/// Benchmarker::Options
///

Benchmarker::Options::Options(unsigned int runs, uint32_t options) : runs(runs), options(options)
{
}

///
/// Benchmarker::Results::Run::toString
///

const std::string Benchmarker::Results::Run::toString::operator()(int value) const
{
    return boost::lexical_cast<std::string>(boost::get<int>(value));
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

Benchmarker::Results::Results(const std::string &name, const SceneConstPtr scene,
                              const PlannerConstPtr planner, const MotionRequestBuilderConstPtr builder,
                              const Options &options)
  : name(name), scene(scene), planner(planner), builder(builder), options(options)
{
    start = IO::getDate();
}

void Benchmarker::Results::addRun(int num, double time, planning_interface::MotionPlanResponse &run)
{
    Run metrics(num, time, run.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if (metrics.success)
        computeMetric(run, metrics);

    runs.emplace_back(metrics);
}

void Benchmarker::Results::computeMetric(planning_interface::MotionPlanResponse &run, Run &metrics)
{
    const robot_trajectory::RobotTrajectory &p = *run.trajectory_;

    if (options.options & MetricOptions::WAYPOINTS)
        metrics.metrics["waypoints"] = (int)p.getWayPointCount();

    if (options.options & MetricOptions::PATH)
        p.getRobotTrajectoryMsg(metrics.path);

    if (options.options & MetricOptions::LENGTH)
        metrics.metrics["length"] = path::getLength(p);

    if (options.options & MetricOptions::CORRECT)
        metrics.metrics["correct"] = path::isCorrect(p, scene);

    if (options.options & MetricOptions::CLEARANCE)
        metrics.metrics["clearance"] = std::get<0>(path::getClearance(p, scene));

    if (options.options & MetricOptions::SMOOTHNESS)
        metrics.metrics["smoothness"] = path::getSmoothness(p);
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

void Benchmarker::benchmark(const std::vector<BenchmarkOutputterPtr> &outputs, const Options &options)
{
    unsigned int count = 0;
    const unsigned int total = requests_.size() * options.runs;

    for (const auto &request : requests_)
    {
        const auto &name = request.first;
        auto &scene = std::get<0>(request.second);
        auto &planner = std::get<1>(request.second);
        const auto &builder = std::get<2>(request.second);
        std::vector<moveit_msgs::RobotTrajectory> trajectories;

        Results results(name, scene, planner, builder, options);

        for (unsigned int j = 0; j < options.runs; ++j)
        {
            ros::WallTime start;

            start = ros::WallTime::now();
            planning_interface::MotionPlanResponse response = planner->plan(scene, builder->getRequest());
            double time = (ros::WallTime::now() - start).toSec();

            results.addRun(j, time, response);
            ROS_INFO("BENCHMARKING: [ %u / %u ] Completed", ++count, total);
        }

        results.finish = IO::getDate();

        for (BenchmarkOutputterPtr output : outputs)
            output->dumpResult(results);
    }
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
        ROS_WARN("These results did not save the path according to the options. Skipping.");
        return;
    }
    const std::string &name = results.name;

    for (Benchmarker::Results::Run run : results.runs)
        bag_.addMessage(name, run.path);
}

///
/// OMPLBenchmarkOutputter
///

OMPLBenchmarkOutputter::OMPLBenchmarkOutputter(const std::string &prefix) : prefix_(prefix)
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
    moveit_msgs::PlanningScene scene_msg;
    const auto &request = results.builder->getRequest();

    results.scene->getSceneConst()->getPlanningSceneMsg(scene_msg);

    YAML::Node yaml;
    yaml["scene"] = IO::toNode(scene_msg);
    yaml["request"] = IO::toNode(request);

    YAML::Emitter yaml_out;
    yaml_out << yaml;

    out << "<<<|" << std::endl;
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
        class toString : public boost::static_visitor<const std::string>
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

        out << name << " " << boost::apply_visitor(toString(), metric.second) << std::endl;
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

    out << "." << std::endl;
    out.close();
}
