#include <boost/math/constants/constants.hpp>

#include <moveit/version.h>

#include <robowflex/robowflex.h>

using namespace robowflex;

Benchmarker::Benchmarker()
{
}

void Benchmarker::addBenchmarkingRequest(const std::string &name, Scene &scene, Planner &planner,
                                         MotionRequestBuilder &request)
{
    requests_.emplace(std::piecewise_construct,     //
                      std::forward_as_tuple(name),  //
                      std::forward_as_tuple(scene, planner, request));
}

void Benchmarker::benchmark(BenchmarkOutputter &output, const Options &options)
{
    unsigned int count = 0;
    const unsigned int total = requests_.size() * options.runs;

    for (const auto &request : requests_)
    {
        const auto &name = request.first;
        const auto &scene = std::get<0>(request.second);
        auto &planner = std::get<1>(request.second);
        const auto &builder = std::get<2>(request.second);

        Results results(name, scene, planner, builder);

        for (int j = 0; j < options.runs; ++j)
        {
            ros::WallTime start;

            start = ros::WallTime::now();
            planning_interface::MotionPlanResponse response = planner.plan(scene, builder.getRequest());
            double time = (ros::WallTime::now() - start).toSec();

            results.addRun(j, time, response);
            ROS_INFO("BENCHMARKING: [ %u / %u ] Completed", ++count, total);
        }

        results.finish = IO::getDate();
        output.dump(results);
    }

    output.close();
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
    metrics.correct = true;
    metrics.length = 0.0;
    metrics.clearance = 0.0;
    metrics.smoothness = 0.0;

    const robot_trajectory::RobotTrajectory &p = *run.trajectory_;
    const planning_scene::PlanningScene &s = *scene.getSceneConst();

    // compute path length
    for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
        metrics.length += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

    // compute correctness and clearance
    collision_detection::CollisionRequest request;
    for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
    {
        collision_detection::CollisionResult result;
        s.checkCollisionUnpadded(request, result, p.getWayPoint(k));

        if (result.collision)
            metrics.correct = false;

        if (!p.getWayPoint(k).satisfiesBounds())
            metrics.correct = false;

        double d = s.distanceToCollisionUnpadded(p.getWayPoint(k));
        if (d > 0.0)  // in case of collision, distance is negative
            metrics.clearance += d;
    }
    metrics.clearance /= (double)p.getWayPointCount();

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

void JSONBenchmarkOutputter::dump(const Benchmarker::Results &results)
{
    if (not is_init)
    {
        outfile_ = IO::createFile(file_);
        outfile_ << "{";
        // TODO: output specific information about the scene and planner structs?

        is_init = true;
    }
    else
    {
        outfile_ << ",";
    }

    outfile_ << "\"" << results.name << "\":[";

    for (size_t i = 0; i < results.runs.size(); i++)
    {
        Benchmarker::Results::Run run = results.runs[i];
        outfile_ << "{";

        outfile_ << "\"name\": \"run_" << run.num << "\",";
        outfile_ << "\"time\":" << run.time << ",";
        outfile_ << "\"success\":" << run.success << ",";
        outfile_ << "\"correct\":" << run.correct << ",";
        outfile_ << "\"length\":" << run.length << ",";
        outfile_ << "\"clearance\":";
        // Check for infinity.
        if (run.clearance == std::numeric_limits<double>::infinity())
            outfile_ << std::numeric_limits<double>::max() << ",";
        else
            outfile_ << run.clearance << ",";
        outfile_ << "\"smoothness\":" << run.smoothness;

        outfile_ << "}";
        // Write the command between each run.
        if (i != results.runs.size() - 1)
            outfile_ << "," << std::endl;
    }

    outfile_ << "]";
}

void JSONBenchmarkOutputter::close()
{
    outfile_ << "}" << std::endl;
    outfile_.close();
}

void OMPLBenchmarkOutputter::dump(const Benchmarker::Results &results)
{
    if (!is_init)
    {
        is_init = true;
        outfile_ = IO::createFile(file_);
    }

    outfile_ << "MoveIt! version " << MOVEIT_VERSION << std::endl;  // version
    outfile_ << "Experiment " << results.name << std::endl;         // experiment
    outfile_ << "Running on " << IO::getHostname() << std::endl;    // hostname
    outfile_ << "Starting at " << IO::getDate() << std::endl;       // date

    // setup
    moveit_msgs::PlanningScene scene_msg;
    const auto &request = results.builder.getRequest();

    results.scene.getSceneConst()->getPlanningSceneMsg(scene_msg);

    YAML::Node yaml;
    yaml["scene"] = scene_msg;
    yaml["request"] = request;

    YAML::Emitter yaml_out;
    yaml_out << yaml;

    outfile_ << "<<<|" << std::endl;
    outfile_ << yaml_out.c_str() << std::endl;
    outfile_ << "|>>>" << std::endl;

    // random seed (fake)
    outfile_ << "0 is the random seed" << std::endl;

    // time limit
    outfile_ << request.allowed_planning_time << " seconds per run" << std::endl;

    // memory limit
    outfile_ << "-1 MB per run" << std::endl;

    // num_runs
    outfile_ << results.runs.size() << " runs per planner" << std::endl;

    // total_time

    auto duration = results.finish - results.start;
    double total = (double)duration.total_milliseconds() / 1000.;
    outfile_ << total << " seconds spent to collect the data" << std::endl;

    // num_enums / enums
    outfile_ << "0 enum types" << std::endl;

    // num_planners
    outfile_ << "1 planners" << std::endl;

    // void Benchmarker::dump(const std::string &file, const Results &results, const Scene &scene, const Planner
    // &planner,
    //                              const MotionRequestBuilder &builder)
    //{
    // metrics["time REAL"] = boost::lexical_cast<std::string>(total_time);
    // metrics["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);

    // metrics["path_" + run.description_[j] + "_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
    // metrics["path_" + run.description_[j] + "_length REAL"] = boost::lexical_cast<std::string>(L);
    // metrics["path_" + run.description_[j] + "_clearance REAL"] = boost::lexical_cast<std::string>(clearance);
    // metrics["path_" + run.description_[j] + "_smoothness REAL"] = boost::lexical_cast<std::string>(smoothness);
    // metrics["path_" + run.description_[j] + "_time REAL"] =
    // boost::lexical_cast<std::string>(run.processing_time_[j]);
    //}
}

void OMPLBenchmarkOutputter::close()
{
}
