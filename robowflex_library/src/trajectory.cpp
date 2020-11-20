/* Author: Constantinos Chamzas, Zachary Kingston */

#include <robowflex_library/trajectory.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <robowflex_library/constants.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

using namespace robowflex;

Trajectory::Trajectory(const RobotConstPtr &robot, const std::string &group)
{
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot->getModelConst(), group);
}

Trajectory::Trajectory(robot_trajectory::RobotTrajectory &trajectory)
{
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory);
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

const robot_trajectory::RobotTrajectoryPtr &Trajectory::getTajectoryConst() const
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

std::size_t Trajectory::size() const
{
    return trajectory_->getWayPointCount();
}

bool Trajectory::computeTimeParameterization(double max_velocity, double max_acceleration)
{
    trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
    return parameterizer.computeTimeStamps(*trajectory_, max_velocity, max_acceleration);
}

void Trajectory::interpolate(unsigned int requestCount)
{
    if (requestCount < this->size() || trajectory_->getWayPointCount() < 2)
        return;

    auto count = requestCount;

    // the remaining length of the path we need to add states along
    double remainingLength = getLength();

    const int n1 = this->size() - 1;
    int added = 0;

    for (int i = 0; i < n1; ++i)
    {
        // new states are constantly added so we need to get the correct pointers every time;
        auto s1 = trajectory_->getWayPointPtr(i + added);
        auto s2 = trajectory_->getWayPointPtr(i + +added + 1);

        // the maximum number of states that can be added on the current motion (without its endpoints)
        // such that we can at least fit the remaining states
        int maxNStates = count + i - this->size();

        if (maxNStates > 0)
        {
            // compute an approximate number of states the following segment needs to contain; this includes
            // endpoints
            double segmentLength = s1->distance(*s2);
            int ns = i + 1 == n1 ? maxNStates + 2 :
                                   (int)floor(0.5 + (double)count * segmentLength / remainingLength) + 1;

            // if more than endpoints are needed
            if (ns > 2)
            {
                ns -= 2;  // subtract endpoints

                // make sure we don't add too many states
                if (ns > maxNStates)
                    ns = maxNStates;

                // compute intermediate states
                for (unsigned int j = 1; j < count; j++)
                {
                    auto state = std::make_shared<robot_state::RobotState>(trajectory_->getRobotModel());
                    double dt = double(j) / double(count);

                    s1->interpolate(*s2, dt, *state);
                    state->update(true);
                    trajectory_->insertWayPoint(i + j, state, dt);
                    // count how many stated have been added
                    added++;
                }
            }
            else
                ns = 0;

            // update what remains to be done
            count -= (ns + 1);
            remainingLength -= segmentLength;
        }
        else
            count--;
    }
    ROS_INFO("Added %d extra states", added);
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

bool Trajectory::isCorrect(const SceneConstPtr &scene) const
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
        double clearance = scene->distanceToCollision(s);
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

