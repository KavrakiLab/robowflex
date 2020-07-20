/* Author: Zachary Kingston */

#include <boost/math/constants/constants.hpp>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <robowflex_library/path.h>
#include <robowflex_library/scene.h>

double robowflex::path::getLength(const robot_trajectory::RobotTrajectory &path, const PathMetric &metric)
{
    double length = 0.0;

    for (std::size_t k = 1; k < path.getWayPointCount(); ++k)
    {
        const auto &s1 = path.getWayPoint(k - 1);
        const auto &s2 = path.getWayPoint(k);

        if (metric)
            length += metric(s1, s2);
        else
            length += s1.distance(s2);
    }

    return length;
}

bool robowflex::path::isCorrect(const robot_trajectory::RobotTrajectory &path, const SceneConstPtr &scene)
{
    bool correct = true;
    auto p = const_cast<robot_trajectory::RobotTrajectory *>(&path);  // HACK: getWayPointPtr is not const...

    for (std::size_t k = 0; k < path.getWayPointCount(); ++k)
    {
        auto s = p->getWayPointPtr(k);
        if (!s->satisfiesBounds())
            return false;

        const auto result = scene->checkCollision(*s);
        if (result.collision)
            return false;
    }

    return correct;
}

std::tuple<double, double, double>
robowflex::path::getClearance(const robot_trajectory::RobotTrajectory &path, const SceneConstPtr &scene)
{
    double minimum = std::numeric_limits<double>::max();
    double maximum = 0;
    double average = 0;
    auto p = const_cast<robot_trajectory::RobotTrajectory *>(&path);  // HACK: getWayPointPtr is not const...

    for (std::size_t k = 0; k < path.getWayPointCount(); ++k)
    {
        const auto &s = p->getWayPointPtr(k);
        double clearance = scene->distanceToCollision(s);
        if (clearance > 0.0)
        {
            average += clearance;
            maximum = (maximum > clearance) ? maximum : clearance;
            minimum = (minimum < clearance) ? minimum : clearance;
        }
    }

    average /= path.getWayPointCount();

    return std::make_tuple(average, minimum, maximum);
}

double robowflex::path::getSmoothness(const robot_trajectory::RobotTrajectory &path, const PathMetric &metric)
{
    double smoothness = 0.0;

    auto distance =
        (metric) ? metric : [](const robot_state::RobotState &a, const robot_state::RobotState &b) {
            return a.distance(b);
        };

    // compute smoothness
    if (path.getWayPointCount() > 2)
    {
        double a = distance(path.getWayPoint(0), path.getWayPoint(1));
        for (std::size_t k = 2; k < path.getWayPointCount(); ++k)
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
            double b = distance(path.getWayPoint(k - 1), path.getWayPoint(k));
            double cdist = distance(path.getWayPoint(k - 2), path.getWayPoint(k));
            double acos_value = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
            if (acos_value > -1.0 && acos_value < 1.0)
            {
                // the smoothness is actually the outside angle of the one we compute
                double angle = (boost::math::constants::pi<double>() - acos(acos_value));

                // and we normalize by the length of the segments
                double u = 2.0 * angle;  /// (a + b);
                smoothness += u * u;
            }

            a = b;
        }

        smoothness /= path.getWayPointCount();
    }

    return smoothness;
}

std::map<std::string, double>
robowflex::path::getFinalPositions(const robot_trajectory::RobotTrajectory &path)
{
    const auto &last = path.getLastWayPoint();

    std::map<std::string, double> map;

    const auto &names = last.getVariableNames();
    const auto &values = last.getVariablePositions();

    for (std::size_t i = 0; i < names.size(); ++i)
        map.emplace(names[i], values[i]);

    return map;
}

bool robowflex::path::computeTimeParameterization(robot_trajectory::RobotTrajectory &path,
                                                  double max_velocity, double max_acceleration)
{
    trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
    return parameterizer.computeTimeStamps(path, max_velocity, max_acceleration);
}
