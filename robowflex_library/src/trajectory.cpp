/* Author: Constantinos Chamzas, Zachary Kingston */

#include <robowflex_library/trajectory.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <robowflex_library/constants.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/yaml.h>

using namespace robowflex;

Trajectory::Trajectory(const RobotConstPtr &robot, const std::string &group)
  : trajectory_(new robot_trajectory::RobotTrajectory(robot->getModelConst(), group))
{
}

Trajectory::Trajectory(const robot_trajectory::RobotTrajectory &trajectory)
  : trajectory_(new robot_trajectory::RobotTrajectory(trajectory))
{
}

Trajectory::Trajectory(robot_trajectory::RobotTrajectoryPtr trajectory)
  : trajectory_(new robot_trajectory::RobotTrajectory(*trajectory))
{
}

void Trajectory::useMessage(const robot_state::RobotState &reference_state,
                            const moveit_msgs::RobotTrajectory &msg)
{
    trajectory_->setRobotTrajectoryMsg(reference_state, msg);
}

void Trajectory::useMessage(const robot_state::RobotState &reference_state,
                            const trajectory_msgs::JointTrajectory &msg)
{
    trajectory_->setRobotTrajectoryMsg(reference_state, msg);
}

bool Trajectory::toYAMLFile(const std::string &filename) const
{
    moveit_msgs::RobotTrajectory msg;
    trajectory_->getRobotTrajectoryMsg(msg);

    YAML::Node node = robowflex::IO::toNode(msg);
    return robowflex::IO::YAMLToFile(node, filename);
}

bool Trajectory::fromYAMLFile(const robot_state::RobotState &reference_state, const std::string &filename)
{
    moveit_msgs::RobotTrajectory msg;
    if (!IO::YAMLFileToMessage(msg, filename))
        return false;

    useMessage(reference_state, msg);
    return true;
}

void Trajectory::addSuffixWaypoint(const robot_state::RobotState &state, double dt)
{
    trajectory_->addSuffixWayPoint(state, dt);
}

const robot_trajectory::RobotTrajectoryPtr &Trajectory::getTrajectoryConst() const
{
    return trajectory_;
}

robot_trajectory::RobotTrajectoryPtr &Trajectory::getTrajectory()
{
    return trajectory_;
}

moveit_msgs::RobotTrajectory Trajectory::getMessage() const
{
    moveit_msgs::RobotTrajectory msg;
    trajectory_->getRobotTrajectoryMsg(msg);
    return msg;
}

std::size_t Trajectory::getNumWaypoints() const
{
    return trajectory_->getWayPointCount();
}

bool Trajectory::computeTimeParameterization(double max_velocity, double max_acceleration)
{
    trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
    return parameterizer.computeTimeStamps(*trajectory_, max_velocity, max_acceleration);
}

bool Trajectory::computeTimeParameterization(robot_trajectory::RobotTrajectory &trajectory,
                                             double max_velocity, double max_acceleration)
{
    trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
    return parameterizer.computeTimeStamps(trajectory, max_velocity, max_acceleration);
}

void Trajectory::interpolate(unsigned int count)
{
#if ROBOWFLEX_AT_LEAST_KINETIC
    if (count < getNumWaypoints() || trajectory_->getWayPointCount() < 2)
        return;

    // the remaining length of the path we need to add states along
    double total_length = getLength();

    // the Number of segments that exist in this path.
    const int n1 = getNumWaypoints() - 1;
    int added = 0;

    for (int seg = 0; seg < n1; ++seg)
    {
        // Last waypoint that has not been interpolated.
        int i = seg + added;
        auto s0 = trajectory_->getWayPointPtr(i);      // First point of the (uninterploated) segment
        auto s2 = trajectory_->getWayPointPtr(i + 1);  // Last point of the (uninterploated) segment

        // compute an approximate number of states the following segment needs to contain; this includes
        // endpoints
        double segment_length = s0->distance(*s2);
        int ns = (int)floor(0.5 + (double)count * segment_length / total_length) + 1;

        // if more than endpoints are needed
        if (ns > 2)
        {
            ns -= 2;  // subtract endpoints

            // compute intermediate states
            for (int j = 1; j < ns; j++)
            {
                // The state to be inserted
                auto s1 = std::make_shared<robot_state::RobotState>(trajectory_->getRobotModel());
                double dt = double(j) / double(ns);

                s0->interpolate(*s2, dt, *s1);
                s1->update(true);
                trajectory_->insertWayPoint(i + j, *s1, dt);
                // count how many stated have been added
                added++;
            }
        }
    }

    RBX_INFO("Added %d extra states in the trajectory", added);
    return;

#endif
    throw Exception(1, "Not Implemented");
}

std::vector<std::vector<double>> Trajectory::vectorize() const
{
    std::vector<std::vector<double>> traj_vec;
    const auto &msg = getMessage();
    for (const auto &p : msg.joint_trajectory.points)
        traj_vec.emplace_back(p.positions);

    return traj_vec;
}

std::vector<std::string> Trajectory::getJointNames() const
{
    return getMessage().joint_trajectory.joint_names;
}

Trajectory &Trajectory::append(const Trajectory &source, double dt, size_t start_index, size_t end_index)
{
    trajectory_->append(*source.getTrajectoryConst(), dt, start_index, end_index);
    return *this;
}

double Trajectory::getLength(const PathMetric &metric) const
{
    double length = 0.0;

    for (std::size_t k = 1; k < trajectory_->getWayPointCount(); ++k)
    {
        const auto &s1 = trajectory_->getWayPoint(k - 1);
        const auto &s2 = trajectory_->getWayPoint(k);

        if (metric)
            length += metric(s1, s2);
        else
            length += s1.distance(s2);
    }

    return length;
}

bool Trajectory::isCollisionFree(const SceneConstPtr &scene) const
{
    bool correct = true;

    for (std::size_t k = 0; k < trajectory_->getWayPointCount(); ++k)
    {
        auto s = trajectory_->getWayPointPtr(k);
        if (!s->satisfiesBounds())
            return false;

        const auto result = scene->checkCollision(*s);
        if (result.collision)
            return false;
    }

    return correct;
}

std::tuple<double, double, double> Trajectory::getClearance(const SceneConstPtr &scene) const
{
    double minimum = std::numeric_limits<double>::max();
    double maximum = 0;
    double average = 0;

    for (std::size_t k = 0; k < trajectory_->getWayPointCount(); ++k)
    {
        const auto &s = trajectory_->getWayPointPtr(k);
        double clearance = scene->distanceToCollision(*s);
        if (clearance > 0.0)
        {
            average += clearance;
            maximum = (maximum > clearance) ? maximum : clearance;
            minimum = (minimum < clearance) ? minimum : clearance;
        }
    }

    average /= trajectory_->getWayPointCount();

    return std::make_tuple(average, minimum, maximum);
}

double Trajectory::getSmoothness(const PathMetric &metric) const
{
    double smoothness = 0.0;

    auto distance =
        (metric) ? metric : [](const robot_state::RobotState &a, const robot_state::RobotState &b) {
            return a.distance(b);
        };

    // compute smoothness
    if (trajectory_->getWayPointCount() > 2)
    {
        double a = distance(trajectory_->getWayPoint(0), trajectory_->getWayPoint(1));
        for (std::size_t k = 2; k < trajectory_->getWayPointCount(); ++k)
        {
            // view the trajectory_->as a sequence of segments, and look at the triangles it forms:
            //          s1
            //          /\          s4
            //      a  /  \ b       |
            //        /    \        |
            //       /......\_______|
            //     s0    c   s2     s3
            //

            // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
            double b = distance(trajectory_->getWayPoint(k - 1), trajectory_->getWayPoint(k));
            double cdist = distance(trajectory_->getWayPoint(k - 2), trajectory_->getWayPoint(k));
            double acos_value = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
            if (acos_value > -1.0 && acos_value < 1.0)
            {
                // the smoothness is actually the outside angle of the one we compute
                double angle = (constants::pi - acos(acos_value));

                // and we normalize by the length of the segments
                double u = 2.0 * angle;  /// (a + b);
                smoothness += u * u;
            }

            a = b;
        }

        smoothness /= trajectory_->getWayPointCount();
    }

    return smoothness;
}

std::map<std::string, double> Trajectory::getFinalPositions() const
{
    const auto &last = trajectory_->getLastWayPoint();

    std::map<std::string, double> map;

    const auto &names = last.getVariableNames();
    const auto &values = last.getVariablePositions();

    for (std::size_t i = 0; i < names.size(); ++i)
        map.emplace(names[i], values[i]);

    return map;
}
