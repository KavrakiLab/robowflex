// =======================================================================
// Copyright 2015 by Ireneusz Szcześniak
// Authors: Ireneusz Szcześniak
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
// =======================================================================

// =======================================================================
// The custom Dijkstra call, which returns the optional path as a list
// of edges along with the cost of the path.  The search is stopped
// when the destination node is reached.
// =======================================================================

#ifndef BOOST_GRAPH_CUSTOM_DIJKSTRA_CALL
#define BOOST_GRAPH_CUSTOM_DIJKSTRA_CALL

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/optional.hpp>
#include <boost/property_map/property_map.hpp>

#include <list>
#include <map>

namespace boost {

  // =======================================================================
  // The function that traces back the result.  The predecessos map
  // should map a vertex to an edge.  Traditional Dijkstra maps a
  // vertex to a vertex.  However, we allow for multigraphs, and so we
  // care about a specific edge that led to a vertex.
  // =======================================================================

  template <typename Graph, typename WeightMap, typename PredMap>
  optional<std::pair<typename WeightMap::value_type,
                     std::list<typename Graph::edge_descriptor> > >
  trace(const Graph &g, WeightMap wm, PredMap pred,
        typename Graph::vertex_descriptor src,
        typename Graph::vertex_descriptor dst)
  {
    typedef typename Graph::vertex_descriptor vertex_descriptor;
    typedef typename Graph::edge_descriptor edge_descriptor;
    typedef typename WeightMap::value_type weight_type;
    typedef typename std::list<typename Graph::edge_descriptor> path_type;
    // The result type.
    typedef typename std::pair<weight_type, path_type> r_type;

    optional<r_type> result;

    if (src == dst)
      result = r_type();
    // Was the solution found?
    else if (pred[dst] != edge_descriptor())
      {
        // The result.
        r_type r;

        // Trace the solution to the source.
        vertex_descriptor c = dst;
        while (c != src)
          {
            const edge_descriptor &e = pred[c];
            // Increase the cost.
            r.first += get(wm, e);
            // Add the edge to the path.
            r.second.push_front(e);
            // Find the predecessing vertex.
            c = source(e, g);
          }

        result = std::move(r);
      }

    return result;
  }

  // =======================================================================
  // That's a helper function that uses BGL's Dijkstra.  There are two
  // important points:
  //
  // * we stop the search where we reach the dst vertex,
  //
  // * we predecessor map associates an edge with a vertex.
  //
  // =======================================================================
  template <typename Graph, typename WeightMap, typename IndexMap,
            typename PredMap>
  void
  stop_dijkstra_at_dst(const Graph &g,
                       typename Graph::vertex_descriptor src,
                       typename Graph::vertex_descriptor dst,
                       WeightMap wm, IndexMap im, PredMap pm)
  {
    // The type of the exception thrown by the cdc_visitor.
    struct exception {};

    struct visitor
    {
      typedef typename Graph::vertex_descriptor vertex_descriptor;
      typedef on_examine_vertex event_filter;
      visitor(vertex_descriptor dst): m_dst(dst) {}
      void operator()(vertex_descriptor v, const Graph& g) {
        if (v == m_dst)
          throw exception();
      }
      vertex_descriptor m_dst;
    };

    auto rep = record_edge_predecessors(pm, on_edge_relaxed());
    auto qat = visitor(dst);
    auto dv = make_dijkstra_visitor(std::make_pair(rep, qat));

    try
      {
        dijkstra_shortest_paths(g, src, weight_map(wm).
                                vertex_index_map(im).visitor(dv));
      }
    catch (exception) {}
  }

  // =======================================================================
  // The function that calls Dijkstra.  The function returns a list of
  // edges of the shortest path.
  // =======================================================================

  template <typename Graph, typename WeightMap, typename IndexMap>
  optional<std::pair<typename WeightMap::value_type,
                     std::list<typename Graph::edge_descriptor> > >
  custom_dijkstra_call(const Graph &g,
                       typename Graph::vertex_descriptor src,
                       typename Graph::vertex_descriptor dst,
                       WeightMap wm, IndexMap im)
  {
    typedef typename Graph::vertex_descriptor vertex_descriptor;
    typedef typename Graph::edge_descriptor edge_descriptor;

    std::map<vertex_descriptor, edge_descriptor> v2e;
    auto pred = make_assoc_property_map(v2e);
    stop_dijkstra_at_dst(g, src, dst, wm, im, pred);
    return trace(g, wm, pred, src, dst);
  }

} // boost

#endif /* BOOST_GRAPH_CUSTOM_DIJKSTRA_CALL */
