/* Author: Carlos Quintero-Peña*/

#include <robowflex_tesseract/stretch.h>
#include <robowflex_tesseract/utils.h>
// #include <robowflex_library/detail/fetch.h>
// #include <robowflex_library/detail/ur5.h>

bool configurePlanner(const std::string &planner_name,
                      const robowflex::RobotPtr &robot,
                      robowflex::PlannerPtr &planner)
{
    if (planner_name == "RRTConnect" or planner_name == "RRTConnectPost")
    {
        planner = std::make_shared<robowflex::OMPL::StretchOMPLPipelinePlanner>(robot, planner_name);

        auto rrtc = static_cast<robowflex::OMPL::StretchOMPLPipelinePlanner *>(planner.get());
        rrtc->initialize();
        return true;
    }

    return false;
}
/*
robot_trajectory::RobotTrajectoryPtr sampleTrajectory(robot_trajectory::RobotTrajectoryPtr &trajectory,
                                                      int num_waypoints)
{
    auto new_traj = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory->getRobotModel(),
                                                                        trajectory->getGroupName());
    auto current_nwp = trajectory->getWayPointCount();

    const robot_state::RobotState &copyi = trajectory->getFirstWayPoint();
    new_traj->addSuffixWayPoint(copyi, 0.0);

    int step = std::floor((current_nwp - 2) / (num_waypoints - 1));

    int i = 0;
    while (i < (num_waypoints - 2))
    {
        const robot_state::RobotState &copy = trajectory->getWayPoint(i * (step + 1));
        new_traj->addSuffixWayPoint(copy, 0.0);
        i++;
    }

    const robot_state::RobotState &copy = trajectory->getLastWayPoint();
    new_traj->addSuffixWayPoint(copy, 0.0);

    return new_traj;
}

std::vector<double> getDistanceToObject(const robot_trajectory::RobotTrajectoryPtr &traj,
                                        const robowflex::ScenePtr &scene, const std::string &object)
{
    int num_wp = traj->getWayPointCount();
    std::vector<double> distances;
    for (unsigned int i = 0; i < num_wp; ++i)
    {
        const auto &st = traj->getWayPointPtr(i);
        auto distance = scene->distanceToObject(st, object);
        distances.push_back(distance);
    }
    return distances;
}

double updatedMean(double x, double mean, int n)
{
    return mean + ((x - mean) / (double)n);
}

double updatedVar(double x, double prev_mean, double new_mean, double var, int n)
{
    return (double)((n == 1) ? (double)0.0 :
                               ((double)(n - 1) * var + (x - prev_mean) * (x - new_mean)) / (double)n);
}

std::vector<double> updateMeanVector(std::vector<double> x, std::vector<double> mean, int n)
{
    int size = mean.size();
    std::vector<double> new_mean(size);
    for (int i = 0; i < size; ++i)
        new_mean[i] = updatedMean(x[i], mean[i], n);

    return new_mean;
}

std::vector<double> updateVarVector(std::vector<double> x, std::vector<double> prev_mean,
                                    std::vector<double> new_mean, std::vector<double> var, int n)
{
    int size = var.size();
    std::vector<double> new_var(size);
    for (int i = 0; i < size; ++i)
        new_var[i] = updatedVar(x[i], prev_mean[i], new_mean[i], var[i], n);

    return new_var;
}

MonteCarloExperiments::MonteCarloExperiments(int waypoint_cnt, int num_experiments, const std::string &path,
                                             const std::string &planner_name)
  : num_experiments_{num_experiments}
{
    fout_.open(path + "_" + planner_name);
    fout_ << planner_name << std::endl;
    results.avg_dist_to_col.resize(waypoint_cnt);
    results.var_dist_to_col.resize(waypoint_cnt);
    initialize();
}

void MonteCarloExperiments::initialize()
{
    cnt_ = 0;
    results.tp_rate = 0.0;
    results.fp_rate = 0.0;
    results.avg_planning_time = 0.0;
    results.var_planning_time = 0.0;
    results.avg_length = 0.0;
    results.var_length = 0.0;
    results.avg_smoothness = 0.0;
    results.var_smoothness = 0.0;
    fill(results.avg_dist_to_col.begin(), results.avg_dist_to_col.end(), 0.0);
    fill(results.var_dist_to_col.begin(), results.var_dist_to_col.end(), 0.0);
}

void MonteCarloExperiments::updatePositivityRates(bool isSampledSceneFeasible)
{
    if (isSampledSceneFeasible)
    {
        results.tp_rate = (results.tp_rate * (cnt_ - 1) + 1) / cnt_;
        results.fp_rate = results.fp_rate * (cnt_ - 1) / cnt_;
    }
    else
    {
        results.fp_rate = (results.fp_rate * (cnt_ - 1) + 1) / cnt_;
        results.tp_rate = results.tp_rate * (cnt_ - 1) / cnt_;
    }
}

void MonteCarloExperiments::updatePlanningTimes(double planning_time)
{
    auto old_avg = results.avg_planning_time;
    results.avg_planning_time = updatedMean(planning_time, old_avg, cnt_);
    results.var_planning_time =
        updatedVar(planning_time, old_avg, results.avg_planning_time, results.var_planning_time, cnt_);
}

void MonteCarloExperiments::updateDistanceToCollisions(const std::vector<double> &distances)
{
    auto old_avg_dis = results.avg_dist_to_col;
    results.avg_dist_to_col = updateMeanVector(distances, old_avg_dis, cnt_);
    results.var_dist_to_col =
        updateVarVector(distances, old_avg_dis, results.avg_dist_to_col, results.var_dist_to_col, cnt_);
}

void MonteCarloExperiments::updateLengths(double length)
{
    auto old_avg = results.avg_length;
    results.avg_length = updatedMean(length, old_avg, cnt_);
    results.var_length = updatedVar(length, old_avg, results.avg_length, results.var_length, cnt_);
}

void MonteCarloExperiments::updateSmoothness(double smoothness)
{
    auto old_avg = results.avg_smoothness;
    results.avg_smoothness = updatedMean(smoothness, old_avg, cnt_);
    results.var_smoothness =
        updatedVar(smoothness, old_avg, results.avg_smoothness, results.var_smoothness, cnt_);
}

void MonteCarloExperiments::increaseCount()
{
    cnt_++;
}

int MonteCarloExperiments::getCount()
{
    return cnt_;
}

void MonteCarloExperiments::setNumExperiments(int num_experiments)
{
    num_experiments_ = num_experiments;
}

bool MonteCarloExperiments::experimentIsOver()
{
    return cnt_ >= num_experiments_;
}

void MonteCarloExperiments::saveResults()
{
    fout_ << cnt_ << " " << results.avg_planning_time << " " << results.var_planning_time << " "
          << results.avg_length << " " << results.var_length << " " << results.avg_smoothness << " "
          << results.var_smoothness << " " << results.tp_rate << " " << results.fp_rate << std::endl;

    for (unsigned int i = 0; i < results.avg_dist_to_col.size(); ++i)
        fout_ << results.avg_dist_to_col.at(i) << " ";
    fout_ << std::endl;

    for (unsigned int i = 0; i < results.var_dist_to_col.size(); ++i)
        fout_ << results.var_dist_to_col.at(i) << " ";
    fout_ << std::endl;
}

void MonteCarloExperiments::printParametersInFile(const Eigen::Vector3d &position,
                                                  const Eigen::Vector3d &orientation)
{
    fout_ << position[0] << " " << position[1] << " " << position[2] << " " << orientation[0] << " "
          << orientation[1] << " " << orientation[2] << " ";
}

void MonteCarloExperiments::close()
{
    fout_.close();
}

*/
