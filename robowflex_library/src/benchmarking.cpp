#include <boost/math/constants/constants.hpp>

#include <rosbag/bag.h>
#include <moveit/version.h>

#include <robowflex_library/robowflex.h>

using namespace robowflex;

Benchmarker::Benchmarker()
{
}

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
        auto scene = std::get<0>(request.second);
        auto planner = std::get<1>(request.second);
        const auto &builder = std::get<2>(request.second);
        std::vector<moveit_msgs::RobotTrajectory> trajectories;

        Results results(name, scene, planner, builder, options);

        for (int j = 0; j < options.runs; ++j)
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

void Benchmarker::Results::addRun(int num, double time, planning_interface::MotionPlanResponse &run)
{
    Run metrics(num, time, run.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if (metrics.success)
        computeMetric(run, metrics);

    runs.emplace_back(metrics);
}

void Benchmarker::Results::computeMetric(planning_interface::MotionPlanResponse &run, Run &metrics)
{
    metrics.waypoints = 0;
    metrics.correct = true;
    metrics.length = 0.0;
    metrics.clearance = 0.0;
    metrics.smoothness = 0.0;

    const robot_trajectory::RobotTrajectory &p = *run.trajectory_;
    const planning_scene::PlanningScene &s = *scene->getSceneConst();

    if (options.run_metric_bits & RunMetricBits::WAYPOINTS)
        metrics.waypoints = p.getWayPointCount();

    if (options.run_metric_bits & RunMetricBits::PATH)
        p.getRobotTrajectoryMsg(metrics.path);

    if (options.run_metric_bits & RunMetricBits::LENGTH)
        // compute path length
        for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
            metrics.length += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

    // compute correctness and clearance
    if ((options.run_metric_bits & RunMetricBits::CORRECT) |
        (options.run_metric_bits & RunMetricBits::CLEARANCE))
    {
        collision_detection::CollisionRequest request;
        for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
        {
            if (options.run_metric_bits & RunMetricBits::CORRECT)
            {
                collision_detection::CollisionResult result;
                s.checkCollisionUnpadded(request, result, p.getWayPoint(k));

                if (result.collision)
                    metrics.correct = false;

                if (!p.getWayPoint(k).satisfiesBounds())
                    metrics.correct = false;
            }

            if (options.run_metric_bits & RunMetricBits::CLEARANCE)
            {
                double d = s.distanceToCollisionUnpadded(p.getWayPoint(k));
                if (d > 0.0)  // in case of collision, distance is negative
                    metrics.clearance += d;
            }
        }
        if (options.run_metric_bits & RunMetricBits::CLEARANCE)
            metrics.clearance /= (double)p.getWayPointCount();
    }

    if (options.run_metric_bits & RunMetricBits::SMOOTHNESS)
    {
        // compute smoothness
        if (p.getWayPointCount() > 2)
        {
            double a = p.getWayPoint(0).distance(p.getWayPoint(1));
            for (std::size_t k = 2; k < p.getWayPointCount(); ++k)
            {
                // view the path as a sequence of segments, and look at the triangles it forms:
                //          s1
                //          /\          s4
                //      a  /  \ b       |
                //        /    \        |
                //       /......\_______|
                //     s0    c   s2     s3
                //

                // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
                double b = p.getWayPoint(k - 1).distance(p.getWayPoint(k));
                double cdist = p.getWayPoint(k - 2).distance(p.getWayPoint(k));
                double acosValue = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
                if (acosValue > -1.0 && acosValue < 1.0)
                {
                    // the smoothness is actually the outside angle of the one we compute
                    double angle = (boost::math::constants::pi<double>() - acos(acosValue));

                    // and we normalize by the length of the segments
                    double u = 2.0 * angle;  /// (a + b);
                    metrics.smoothness += u * u;
                }
                a = b;
            }
            metrics.smoothness /= (double)p.getWayPointCount();
        }
    }
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

    uint32_t bitmask = results.options.run_metric_bits;
    for (size_t i = 0; i < results.runs.size(); i++)
    {
        Benchmarker::Results::Run run = results.runs[i];
        outfile_ << "{";

        outfile_ << "\"name\": \"run_" << run.num << "\",";
        outfile_ << "\"time\":" << run.time << ",";
        outfile_ << "\"success\":" << run.success; 
        if (bitmask & Benchmarker::RunMetricBits::CORRECT)
            outfile_ << ",\"correct\":" << run.correct;
        if (bitmask & Benchmarker::RunMetricBits::LENGTH)
            outfile_ << ",\"length\":" << run.length;
        
        if (bitmask & Benchmarker::RunMetricBits::CLEARANCE)
        {
            outfile_ << ",\"clearance\":";
            // Check for infinity.
            if (run.clearance == std::numeric_limits<double>::infinity())
                outfile_ << std::numeric_limits<double>::max();
            else
                outfile_ << run.clearance;
        }
        if (bitmask & Benchmarker::RunMetricBits::SMOOTHNESS)
            outfile_ << ",\"smoothness\":" << run.smoothness;

        outfile_ << "}";
        // Write the command between each run.
        if (i != results.runs.size() - 1)
            outfile_ << "," << std::endl;
    }

    outfile_ << "]";
}

JSONBenchmarkOutputter::~JSONBenchmarkOutputter()
{
    outfile_ << "}" << std::endl;
    outfile_.close();
}

void TrajectoryBenchmarkOutputter::dumpResult(const Benchmarker::Results &results)
{
    if (!(results.options.run_metric_bits & Benchmarker::RunMetricBits::PATH))
    {
        ROS_WARN("These results did not save the path according to the options. Skipping.");
        return;
    }
    const std::string &name = results.name;

    for (Benchmarker::Results::Run run : results.runs)
        bag_.addMessage(name, run.path);
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
    yaml["scene"] = scene_msg;
    yaml["request"] = request;

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
    // Get the number of what metrics we actually saved by counting the bits set to true.
    // First, ignore the PATH bit, but look at all the other metrics we saved.
    uint32_t bitmask = results.options.run_metric_bits &
                  (Benchmarker::RunMetricBits::ALL & !Benchmarker::RunMetricBits::PATH);
    uint32_t count;
    for (count=0; bitmask; count += 1)
        bitmask &= bitmask - 1;
    bitmask = results.options.run_metric_bits;
    out << count + 2 << " properties for each run" << std::endl;  // run_properties
    out << "time REAL" << std::endl;
    out << "success BOOLEAN" << std::endl;
    if (bitmask & Benchmarker::RunMetricBits::WAYPOINTS)
        out << "waypoints INTEGER" << std::endl;
    if (bitmask & Benchmarker::RunMetricBits::CORRECT)
        out << "correct BOOLEAN" << std::endl;
    if (bitmask & Benchmarker::RunMetricBits::LENGTH)
        out << "length REAL" << std::endl;
    if (bitmask & Benchmarker::RunMetricBits::CLEARANCE)
        out << "clearance REAL" << std::endl;
    if (bitmask & Benchmarker::RunMetricBits::SMOOTHNESS)
        out << "smoothness REAL" << std::endl;

    out << results.runs.size() << " runs" << std::endl;

    for (auto &run : results.runs)
    {
        out << run.time << "; "        //
            << run.success << "; ";
        if (bitmask & Benchmarker::RunMetricBits::WAYPOINTS)
            out << run.waypoints << "; ";
        if (bitmask & Benchmarker::RunMetricBits::CORRECT)
            out << run.correct << "; ";
        if (bitmask & Benchmarker::RunMetricBits::LENGTH)
            out << run.length << "; ";
        if (bitmask & Benchmarker::RunMetricBits::CLEARANCE)
            out << run.clearance << "; ";
        if (bitmask & Benchmarker::RunMetricBits::SMOOTHNESS)
            out << run.smoothness << "; ";
        out << std::endl;
    }

    out << "." << std::endl;
    out.close();
}
