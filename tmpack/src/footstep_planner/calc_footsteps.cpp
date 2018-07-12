#ifndef CALC_FOOTSTEPS_CPP
#define CALC_FOOTSTEPS_CPP

#include <gflags/gflags.h>
#include <stdlib.h>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iostream>
#include <utility>
#include <vector>
#include "utils/geom_2D.h"
#include "utils/util.h"

namespace footstep_planning
{
    DEFINE_double(walk_centroid, 40 * 2.54, "The (scaled) \"optimal\" step distance.");
    DEFINE_double(step_max_dist, 75 * 2.54, "The (scaled) maximum step distance.");
    DEFINE_double(step_qual_weight, 1.0, "The weight to give step quality when building the graph.");
    DEFINE_double(num_step_weight, 1000.0, "The weight to give the number of steps when building the graph.");

    DEFINE_double(yaw_range, M_PI / 2,
                  "The max angular distance the feet can be placed from the "
                  "torso's yaw angle.");

    template <class PredecessorMap>
    class record_predecessors : public boost::dijkstra_visitor<>
    {
    public:
        record_predecessors(PredecessorMap p) : m_predecessor(p)
        {
        }

        template <class Edge, class Graph>
        void edge_relaxed(Edge e, Graph &g)
        {
            // set the parent of the target(e) to source(e)
            put(m_predecessor, target(e, g), source(e, g));
        }

    protected:
        PredecessorMap m_predecessor;
    };

    template <class PredecessorMap>
    record_predecessors<PredecessorMap> make_predecessor_recorder(PredecessorMap p)
    {
        return record_predecessors<PredecessorMap>(p);
    }

    // TODO: A cool heuristic for footstep planning.
    // I think you don't really need to consider the yaw and
    // moving foot because the ankles can rotate freely.
    double eval_step(point_2D p1, point_2D p2, foot moving_foot, double start_yaw)

    {
        return (FLAGS_walk_centroid - dist(p1, p2)) * (FLAGS_walk_centroid - dist(p1, p2)) *
                   FLAGS_step_qual_weight +
               1.0 * FLAGS_num_step_weight + (dist(p1, p2) > FLAGS_step_max_dist ? 1000000000 : 0);
    }
    // Vertex i cooresponds to point(i)
    // try to prevent rotating feet
    // cache IK solutions for pairs of steps

    // TODO: A better way to choose foot placements based on torso pose
    struct torsoFootPlacementComparator
    {
        const point_2D &torso_pose;
        bool operator()(point_2D i, point_2D j)
        {
            return torsoFootPlacementEval(i) < torsoFootPlacementEval(j);
        }

        double torsoFootPlacementEval(point_2D i)
        {
            return (dist(i, torso_pose) - FLAGS_step_max_dist / 2) *
                   (dist(i, torso_pose) - FLAGS_step_max_dist / 2);
        }

        torsoFootPlacementComparator(const point_2D &torso_pose) : torso_pose(torso_pose)
        {
        }
    };

    // Returns the indices of the points that should be used with
    // the given torso location
    std::vector<point_2D> getFootPlacementsFromTorsoPose(std::vector<point_2D> points, point_2D torso_point,
                                                         double torso_yaw)
    {
        std::vector<point_2D> right_points;
        std::vector<point_2D> left_points;

        for (size_t i = 0; i < points.size(); i++)
        {
            if (dist(torso_point, points[i]) < FLAGS_step_max_dist / 2)
            {
                double angle = atan2(points[i].y - torso_point.y, points[i].x - torso_point.x);
                angle = normalizeAngle(torso_yaw - angle);
                if (angle < normalizeAngle(M_PI / 2 + FLAGS_yaw_range) &&
                    angle > normalizeAngle(M_PI / 2 - FLAGS_yaw_range))
                {
                    right_points.push_back(points[i]);
                }
                else if (angle > normalizeAngle(-M_PI / 2 - FLAGS_yaw_range) &&
                         angle < normalizeAngle(-M_PI / 2 + FLAGS_yaw_range))
                {
                    left_points.push_back(points[i]);
                }
            }
        }
        std::cout << "number of left points: " << left_points.size() << std::endl;
        std::cout << "number of right points: " << right_points.size() << std::endl;

        torsoFootPlacementComparator myComparator(torso_point);
        std::vector<point_2D> return_points;

        double best_pair_score = 1000000000;
        point_2D left_point(0, 0);
        point_2D right_point(0, 0);

        for (auto p1 : right_points)
        {
            for (auto p2 : left_points)
            {
                if (myComparator.torsoFootPlacementEval(p1) + myComparator.torsoFootPlacementEval(p2) <
                    best_pair_score)
                {
                    best_pair_score =
                        myComparator.torsoFootPlacementEval(p1) + myComparator.torsoFootPlacementEval(p2);
                    left_point = p2;
                    right_point = p1;
                }
            }
        }

        std::cout << left_point << std::endl;
        std::cout << right_point << std::endl;

        return_points.push_back(left_point);
        return_points.push_back(right_point);

        return return_points;
    }

    class FootstepPlanner
    {
        Graph g;

        Graph foot_graph;

    public:
        void buildGraph(std::vector<point_2D> points)
        {
            for (size_t i = 0; i < points.size(); i++)
            {
                for (int j = 0; j < points.size(); j++)
                {
                    if (i != j)
                    {
                        point_2D p = points[i];
                        point_2D p2 = points[j];
                        boost::add_edge(i, j, eval_step(p, p2, foot::left, 0), g);

                        boost::add_edge(2 * i, 2 * j + 1, eval_step(p, p2, foot::left, 0), foot_graph);
                        boost::add_edge(2 * i + 1, 2 * j, eval_step(p, p2, foot::left, 0), foot_graph);
                    }
                }
            }
        }

        // From the torso pose we can back out pairs of foot placements
        // that may work. We know which foot we start with and we use end_foot
        // to ensure we finish facing the correct direction.
        std::vector<point_2D> calculateFootPlacementsForTorso(std::vector<point_2D> points, point_2D start,
                                                              point_2D torso_goal_point,
                                                              double torso_goal_yaw, foot start_foot)
        {
            std::vector<point_2D> final_feet_placements =
                getFootPlacementsFromTorsoPose(points, torso_goal_point, torso_goal_yaw);

            std::vector<point_2D> seq1 =
                calculateFootPlacements(points, start, final_feet_placements[0], start_foot, !start_foot);
            std::vector<point_2D> seq2 =
                calculateFootPlacements(points, start, final_feet_placements[1], start_foot, !start_foot);
            std::cout << "seq1: " << seq1.size() << std::endl;
            std::cout << "seq2: " << seq2.size() << std::endl;

            if (seq1.size() < seq2.size())
            {
                seq1.push_back(final_feet_placements[1]);
                return seq1;
            }
            else
            {
                seq2.push_back(final_feet_placements[0]);
                return seq2;
            }
        }

        std::vector<point_2D> calculateFootPlacements(std::vector<point_2D> points, point_2D start,
                                                      point_2D goal, foot start_foot, foot end_foot)
        {
            if (boost::num_edges(g) == 0)
            {
                std::cout << "The graph has no edges. Did you forget to call buildGraph?" << std::endl;

                buildGraph(points);
            }

            std::vector<point_2D> foot_steps;

            // Find the indicies of the start and goal points
            int startI, goalI;
            for (size_t i = 0; i < points.size(); i++)
            {
                if (points[i] == start)
                    startI = i;
                if (points[i] == goal)
                    goalI = i;
            }

            foot_steps.clear();

            if (start_foot == foot::left)
            {
                startI = 2 * startI;
            }
            else
            {
                startI = 2 * startI + 1;
            }

            if (end_foot == foot::left)
            {
                goalI = 2 * goalI;
            }
            else
            {
                goalI = 2 * goalI + 1;
            }

            // Run Djikstra on our weighted graph to find the optimal foot
            // placements
            std::vector<int> d(num_vertices(foot_graph));
            std::vector<Vertex> p(boost::num_vertices(foot_graph),
                                  boost::graph_traits<Graph>::null_vertex());  // the predecessor
                                                                               // array
            boost::dijkstra_shortest_paths(
                foot_graph, startI, boost::distance_map(&d[0]).visitor(make_predecessor_recorder(&p[0])));

            // walk up the tree to get our path
            int vi = goalI;
            while (vi != startI)
            {
                foot_steps.push_back(points[vi]);
                vi = p[vi];
            }
            // include the start point
            foot_steps.push_back(points[vi]);

            // walking up the tree means the path is backwards
            std::reverse(foot_steps.begin(), foot_steps.end());

            return foot_steps;
        }
    };

}  // namespace footstep_planning

#endif
