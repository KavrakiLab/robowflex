/* Author: Carlos Quintero-Peña*/

#ifndef CARLOS_UTILS_
#define CARLOS_UTILS_

// #include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
// #include <robowflex_library/builder.h>
#include <fstream>

bool configurePlanner(const std::string &planner_name,
                      const robowflex::RobotPtr &robot,
                      robowflex::PlannerPtr &planner);
/*
robot_trajectory::RobotTrajectoryPtr sampleTrajectory(robot_trajectory::RobotTrajectoryPtr &trajectory,
                                                      int num_waypoints);

std::vector<double> getDistanceToObject(const robot_trajectory::RobotTrajectoryPtr &traj,
                                        const robowflex::ScenePtr &scene, const std::string &object);


class MonteCarloExperiments
{
public:
    struct mce_result
    {
        double tp_rate;
        double fp_rate;
        double avg_planning_time;
        double var_planning_time;
        std::vector<double> avg_dist_to_col;
        std::vector<double> var_dist_to_col;
        double avg_length;
        double var_length;
        double avg_smoothness;
        double var_smoothness;
    } results;

    MonteCarloExperiments(int waypoint_cnt, int num_experiments, const std::string &filePath,
                          const std::string &planner_name);

    void initialize();

    void updatePositivityRates(bool isSampledSceneFeasible);

    void updatePlanningTimes(double planning_time);

    void updateDistanceToCollisions(const std::vector<double> &distances);

    void updateLengths(double length);

    void updateSmoothness(double smoothness);

    void increaseCount();

    int getCount();

    void setNumExperiments(int num_experiments);

    bool experimentIsOver();

    void saveResults();

    void printParametersInFile(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    void close();

private:
    int cnt_{0};
    int num_experiments_{0};
    std::ofstream fout_;
};*/

#endif
