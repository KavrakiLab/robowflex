/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Andrew Wells */

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
#include "utils/yen_ksp.hpp"

#define BIG_NUM 1000000000

#define FLAGS_walk_centroid (40*2.54)
#define FLAGS_step_max_dist (75*2.54)
#define FLAGS_step_qual_weight (1.0)
#define FLAGS_num_step_weight (1000.0)
#define FLAGS_yaw_range (M_PI / 2)


namespace footstep_planning {
// DEFINE_double(walk_centroid, 40 * 2.54,
//               "The (scaled) \"optimal\" step distance.");
// DEFINE_double(step_max_dist, 75 * 2.54, "The (scaled) maximum step distance.");
// DEFINE_double(step_qual_weight, 1.0,
//               "The weight to give step quality when building the graph.");
// DEFINE_double(
//     num_step_weight, 1000.0,
//     "The weight to give the number of steps when building the graph.");

// DEFINE_double(yaw_range, M_PI / 2,
//               "The max angular distance the feet can be placed from the "
//               "torso's yaw angle.");

template <class PredecessorMap>
class record_predecessors : public boost::dijkstra_visitor<> {
 public:
  record_predecessors(PredecessorMap p) : m_predecessor(p) {}

  template <class Edge, class Graph>
  void edge_relaxed(Edge e, Graph &g) {
    // set the parent of the target(e) to source(e)
    put(m_predecessor, target(e, g), source(e, g));
  }

 protected:
  PredecessorMap m_predecessor;
};

template <class PredecessorMap>
record_predecessors<PredecessorMap> make_predecessor_recorder(
    PredecessorMap p) {
  return record_predecessors<PredecessorMap>(p);
}

// TODO: A cool heuristic for footstep planning.
// I think you don't really need to consider the yaw and
// moving foot because the ankles can rotate freely.
double eval_step(point_2D p1, point_2D p2, foot moving_foot,
                 double start_yaw) {
  return (FLAGS_walk_centroid - dist(p1, p2)) *
             (FLAGS_walk_centroid - dist(p1, p2)) * FLAGS_step_qual_weight +
         1.0 * FLAGS_num_step_weight +
         (dist(p1, p2) > FLAGS_step_max_dist ? BIG_NUM : 0);
}
// Vertex i cooresponds to point(i)
// try to prevent rotating feet
// cache IK solutions for pairs of steps

// TODO: A better way to choose foot placements based on torso pose
struct torsoFootPlacementComparator {
  const point_2D &torso_pose;
  bool operator()(std::pair<point_2D, point_2D> i, std::pair<point_2D, point_2D> j) {
    return torsoFootPlacementEval(i) < torsoFootPlacementEval(j);
  }
  
  //TODO: Evaluate the angle when choosing feet placements
  double torsoFootPlacementEval(std::pair<point_2D, point_2D> i) {
    return (dist(i.first, torso_pose) - FLAGS_step_max_dist / 2) *
           (dist(i.first, torso_pose) - FLAGS_step_max_dist / 2) +
           (dist(i.second, torso_pose) - FLAGS_step_max_dist / 2) *
           (dist(i.second, torso_pose) - FLAGS_step_max_dist / 2);
  }

  torsoFootPlacementComparator(const point_2D &torso_pose)
      : torso_pose(torso_pose) {}
};

// Returns sorted vector of pairs of points. Pairs are given as <left, right>
std::vector<std::pair<point_2D, point_2D>> getFootPlacementsFromTorsoPose(
    std::vector<point_2D> points, point_2D torso_point, double torso_yaw) {
  std::vector<point_2D> right_points;
  std::vector<point_2D> left_points;

  for (size_t i = 0; i < points.size(); i++) {
    if (dist(torso_point, points[i]) < FLAGS_step_max_dist / 2) {
      double angle =
          atan2(points[i].y - torso_point.y, points[i].x - torso_point.x);
      angle = normalizeAngle(torso_yaw - angle);
      if (angle < normalizeAngle(M_PI / 2 + FLAGS_yaw_range) &&
          angle > normalizeAngle(M_PI / 2 - FLAGS_yaw_range)) {
        right_points.push_back(points[i]);
      } else if (angle > normalizeAngle(-M_PI / 2 - FLAGS_yaw_range) &&
                 angle < normalizeAngle(-M_PI / 2 + FLAGS_yaw_range)) {
        left_points.push_back(points[i]);
      }
    }
  }
  std::cout << "number of left points: " << left_points.size() << std::endl;
  std::cout << "number of right points: " << right_points.size() << std::endl;

  std::vector<std::pair<point_2D, point_2D>> combined_points;

  for(auto r : right_points) {
    for(auto l : left_points) {
      combined_points.push_back(std::make_pair(l, r));
    }
  }

  torsoFootPlacementComparator myComparator(torso_point);
  std::sort(combined_points.begin(), combined_points.end(), myComparator);

  return combined_points;
}

class FootstepPlanner {
  public:
  Graph foot_graph;

 public:
  void buildGraph(std::vector<point_2D> points) {
    for (size_t i = 0; i < points.size(); i++) {
      for (size_t j = i; j < points.size(); j++) {
        if (i != j) {
          point_2D p = points[i];
          point_2D p2 = points[j];
          if(eval_step(p, p2, footstep_planning::foot::left, 0) < BIG_NUM) {
            boost::add_edge(2 * i, 2 * j + 1, eval_step(p, p2, foot::left, 0),
                          foot_graph);
            boost::add_edge(2 * i + 1, 2 * j, eval_step(p, p2, foot::left, 0),
                          foot_graph);
          }
        }
      }
    }
  }

  // Returns n (default 1) paths for each pair of <left, right> final
  // placements found for the torso pose.
  // From the torso pose we can back out pairs of foot placements
  // that may work. We know which foot we start with and we use end_foot
  // to ensure we finish facing the correct direction.
  std::vector<std::vector<point_2D>> calculateFootPlacementsForTorso(
      std::vector<point_2D> points, point_2D start, point_2D torso_goal_point,
      double torso_goal_yaw, foot start_foot, size_t num_paths_per_placement = 1) {

    std::vector<std::vector<point_2D>> walks;

    std::vector<std::pair<point_2D,point_2D>> final_feet_placements =
        getFootPlacementsFromTorsoPose(points, torso_goal_point,
                                       torso_goal_yaw);


    for(auto p : final_feet_placements) {
      //Each pair is <left_index, right_index> we push two paths:
      //left-foot-last and right-foot-last
      std::vector<std::vector<point_2D>> seqs1 = calculateNFootPlacements(
          points, start, p.first, start_foot, foot::left, num_paths_per_placement);
      // seq1.push_back(p.second);
      for(auto seq : seqs1) {
        seq.push_back(p.second);
        std::cout<<"path length left: "<<seq.size()<<std::endl;
        walks.push_back(seq);
      }

      std::vector<std::vector<point_2D>> seqs2 = calculateNFootPlacements(
          points, start, p.second, start_foot, foot::right, num_paths_per_placement);
      // seq2.push_back(p.first);
      for(auto seq : seqs2) {
        seq.push_back(p.first);
        std::cout<<"path length right: "<<seq.size()<<std::endl;
        walks.push_back(seq);
      }
    }
    return walks;
  }

// returns pair of indices: <start_index, goal_index>
  std::pair<size_t, size_t> getStartAndGoalIndices(std::vector<point_2D> points,
                                                point_2D start, point_2D goal,
                                                foot start_foot,
                                                foot end_foot) {
    size_t startI = -1, goalI = -1;
    for (size_t i = 0; i < points.size(); i++) {
      if (points[i] == start) startI = i;
      if (points[i] == goal) goalI = i;
    }

    assert(startI >= 0 && goalI >= 0);

    if (start_foot == foot::left) {
      startI = 2 * startI;
    } else {
      startI = 2 * startI + 1;
    }

    if (end_foot == foot::left) {
      goalI = 2 * goalI;
    } else {
      goalI = 2 * goalI + 1;
    }

    return std::make_pair(startI, goalI);
  }

  size_t getPointIndexFromVertex(const size_t i, const foot &foot) {
    if(foot == footstep_planning::foot::left) {
      assert(i%2 == 0);
      return i/2;
    }
    else {
      assert(i%2 == 1);
      return (i-1)/2;
    }
  }

  std::vector<point_2D> calculateFootPlacements(std::vector<point_2D> points,
                                                point_2D start, point_2D goal,
                                                foot start_foot,
                                                foot end_foot) {
    return calculateNFootPlacements(points, start, goal, start_foot, end_foot)[0];
  }

  // Returns n (default 1) paths from start to goal
  std::vector<std::vector<point_2D>> calculateNFootPlacements(std::vector<point_2D> points,
                                                point_2D start, point_2D goal,
                                                foot start_foot,
                                                foot end_foot,
                                                size_t num_paths = 1) {
    if (boost::num_edges(foot_graph) == 0) {
      std::cout << "The graph has no edges. Did you forget to call buildGraph?"
                << std::endl;
      buildGraph(points);
    }

    std::pair<size_t, size_t> start_and_goal = getStartAndGoalIndices(points, start, goal, start_foot, end_foot);
    size_t startI = start_and_goal.first;
    size_t goalI = start_and_goal.second;

    std::list<std::pair<double, std::list<typename Graph::edge_descriptor>>> pairs = yen_ksp(foot_graph, startI, goalI, num_paths);

    std::vector<std::vector<point_2D>> return_paths;
    for(auto pr : pairs) {
      std::list<typename Graph::edge_descriptor> path = pr.second;
      std::list<point_2D> points_on_path;
      foot curr_foot = start_foot;
      for(auto ed : path) {
        size_t i = boost::source(ed, foot_graph);
        points_on_path.push_back(points[getPointIndexFromVertex(i, curr_foot)]);
        curr_foot = !curr_foot;
      }
      size_t i = boost::target(path.back(), foot_graph);
      points_on_path.push_back(points[getPointIndexFromVertex(i, curr_foot)]);

      return_paths.push_back(std::vector<point_2D>());

      std::cout<<"points on path: "<<points_on_path.size()<<std::endl;


      std::copy( points_on_path.begin(), points_on_path.end(), std::back_inserter( return_paths.back() ) );
    }

    return return_paths;
  }

};

}  // namespace footstep_planning

#endif
