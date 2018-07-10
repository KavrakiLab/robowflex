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
    double eval_step(point_2D p1, point_2D p2)
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
            return (dist(i, torso_pose) - FLAGS_step_max_dist / 2) *
                       (dist(i, torso_pose) - FLAGS_step_max_dist / 2) <
                   (dist(j, torso_pose) - FLAGS_step_max_dist / 2) *
                       (dist(j, torso_pose) - FLAGS_step_max_dist / 2);
        }
        torsoFootPlacementComparator(const point_2D &torso_pose) : torso_pose(torso_pose)
        {
        }
    };

    // Returns the indices of the points that should be used with
    // the given torso location
    std::vector<point_2D> getFootPlacementsFromTorsoPose(std::vector<point_2D> points, point_2D torso_point)
    {
        std::vector<point_2D> tmp_points;
        for (size_t i = 0; i < points.size(); i++)
        {
            if (dist(torso_point, points[i]) < FLAGS_step_max_dist / 2)
            {
                tmp_points.push_back(points[i]);
            }
        }
        std::cout << "number of candidate points: " << tmp_points.size() << std::endl;

        // Take the best point we found
        torsoFootPlacementComparator myComparator(torso_point);
        std::sort(tmp_points.begin(), tmp_points.end(), myComparator);
        std::vector<point_2D> return_points;
        return_points.push_back(tmp_points[0]);

        // Take the best match to that point
        size_t best_match = 1;
        double best_match_score = 10000000000;
        for (size_t i = 1; i < tmp_points.size(); i++)
        {
            if (eval_step(tmp_points[0], tmp_points[i]) <= best_match_score)
            {
                eval_step(tmp_points[0], tmp_points[i]);
                best_match = i;
            }
        }
        return_points.push_back(tmp_points[best_match]);

        return return_points;
    }

    class FootstepPlanner
    {
        Graph g;

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
                        boost::add_edge(i, j, eval_step(p, p2), g);
                    }
                }
            }
        }

        std::vector<point_2D> calculateFootPlacementsForTorso(std::vector<point_2D> points, point_2D start,
                                                              point_2D torso_goal, foot start_foot)
        {
            std::vector<point_2D> final_feet_placements = getFootPlacementsFromTorsoPose(points, torso_goal);

            std::vector<point_2D> seq1 =
                calculateFootPlacements(points, start, final_feet_placements[0], start_foot);
            std::vector<point_2D> seq2 =
                calculateFootPlacements(points, start, final_feet_placements[1], start_foot);
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
                                                      point_2D goal, foot start_foot)
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

            // Run Djikstra on our weighted graph to find the optimal foot
            // placements
            std::vector<int> d(num_vertices(g));
            std::vector<Vertex> p(boost::num_vertices(g),
                                  boost::graph_traits<Graph>::null_vertex());  // the predecessor
                                                                               // array
            boost::dijkstra_shortest_paths(
                g, startI, boost::distance_map(&d[0]).visitor(make_predecessor_recorder(&p[0])));

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
