// =======================================================================
// Copyright 2015 by Ireneusz Szcześniak
// Author: Ireneusz Szcześniak <www.irkos.org>
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
// =======================================================================

// =======================================================================
// This is the implementation of the Yen algorithm:
//
// Jin Y. Yen, Finding the k shortest loopless paths in a network,
// Management Science, vol. 17, no. 11, July 1971, pages 712-716
//
// Note 1: in the article there are Q^k Q_k Q^k_k used.  As far as I
// can tell they have the same meaning.  Q simply denotes the number
// of the node previous to the last node in a path (which always
// equals to the number of nodes in the path - 1), while (Q) denotes
// the node number Q in a path.  For instance, in the path a-b-c-d,
// the number of the node previous to the last is Q = 3, while (Q) =
// c.  Q^k refers to the k-th shortest path, and simply means the
// number of the node, which is previous to the last in the k-shortest
// path.
// =======================================================================

#ifndef BOOST_GRAPH_YEN_KSP
#define BOOST_GRAPH_YEN_KSP

#include <list>
#include <set>
#include <utility>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/optional.hpp>

#include <tmpack/utils/custom_dijkstra_call.hpp>

namespace boost {

  template <typename G>
  using Edge = typename G::edge_descriptor;

  template <typename G>
  using Vertex = typename G::vertex_descriptor;

  template <typename G>
  using Path = std::list<Edge<G>>;
  
  template <typename W, typename G>
  using Result = std::pair<W, Path<G>>;

  template <typename Graph, typename WeightMap, typename IndexMap>
  bool
  yen_ksp(const Graph& g, Vertex<Graph> s, Vertex<Graph> t,
          WeightMap wm, IndexMap im,
          std::list<Result<typename WeightMap::value_type, Graph>> &A,
          std::set<Result<typename WeightMap::value_type, Graph>> &B)
  {
    using vs_type = std::set<Vertex<Graph>>;
    using es_type = std::set<Edge<Graph>>;
    using kr_type = Result<typename WeightMap::value_type, Graph>;
    using fg_type = filtered_graph<Graph,
                                   is_not_in_subset<es_type>,
                                   is_not_in_subset<vs_type>>;

    optional<kr_type> ksp;

    // If A is empty, then we're looking for the first shortest path.
    if (A.empty())
      {
        assert(B.empty());
        // Try to find the (optional) shortest path.
        ksp = custom_dijkstra_call(g, s, t, wm, im);
      }
    else
      {
        // The previous shortest result and path.
        const auto &psr = A.back();
        const auto &prev_shortest_path = psr.second;

        // The set of excluded edges.  It's a set, because the
        // algorithm can try to exclude an edge many times.
        es_type excluded_edges;
        // The set of excluded vertexes.
        vs_type excluded_vertices;

        // The edge predicate.
        is_not_in_subset<es_type> edge_predicate(excluded_edges);
        // The vertex predicate.
        is_not_in_subset<vs_type> vertex_predicate(excluded_vertices);

        // The filtered graph.
        fg_type filtered_g(g, edge_predicate, vertex_predicate);

        // The root result.
        kr_type rr;
        // The root path.
        const Path<Graph> &rp = rr.second;


        //TODO: This won't work because the next shortest path isn't guaranteed to be a
        //variant of the next to last shortest path. We need to consider all
        //previous shortest paths

        // Use the previous shortest path to get tentative paths.  We
        // can go ahead with the loop without checking any condition
        // (the condition in the for-statement is true): the path
        // found must have at least one link, because s != t.
        for(typename Path<Graph>::const_iterator i = prev_shortest_path.begin();
            true;)
          {
            // An edge of the previous shortest path.
            const Edge<Graph> &edge = *i;

            // The spur vertex - we try to deviate at this node.
            const Vertex<Graph> &sv = source(edge, g);

            // Iterate over all previous shortest paths.  An iteration
            // examines j-th shortest path.
            for(const auto &jr: A)
              {
                // The j-th shortest path.
                const Path<Graph> &jp = jr.second;

                // Let's prepare for the comparison.
                typename Path<Graph>::const_iterator jpi = jp.begin();
                typename Path<Graph>::const_iterator rpi = rp.begin();

                // Iterate as long as the edges are equal.
                while(jpi != jp.end() && rpi != rp.end() && *jpi == *rpi)
                  ++jpi, ++rpi;

                // Make sure we didn't reach the end of jp.  If we
                // did, there is no next edge in jp, which we could
                // exclude.  Also, make sure we reached the end of rp,
                // i.e., the jp begins with the complete rp, and not a
                // head of rp.
                if (jpi != jp.end() && rpi == rp.end())
                  excluded_edges.insert(*jpi);
              }

            // Optional spur result.
            optional<kr_type>
              osr = custom_dijkstra_call(filtered_g, sv, t, wm, im);

            if (osr)
              {
                // The tentative result.
                kr_type tr = std::move(osr.get());
                tr.first += rr.first;
                tr.second.insert(tr.second.begin(), rp.begin(),
                                 rp.end());
                B.insert(std::move(tr));
              }

            // We have the condition to break the loop here, and not
            // at the beginning, because we don't want to execute the
            // remainer of the loop in vain.
            if (++i == prev_shortest_path.end())
              break;

            // Remove the vertex that in this iteration is the spur,
            // but in the next iteration it's going to be a vertex
            // that should not be considered in the search for a spur
            // path.
            excluded_vertices.insert(sv);

            // Add the edge to the back of the root result.
            rr.first += get(wm, edge);
            rr.second.push_back(edge);
          }

        // Take the shortest tentative path and make it the next
        // shortest path.
        if (!B.empty())
          {
            ksp = *B.begin();
            B.erase(B.begin());
          }
      }
  
    if (ksp)
      A.push_back(std::move(ksp.get()));
          
    return bool(ksp);
  }
  
  template <typename Graph, typename WeightMap, typename IndexMap>
  std::list<std::pair<typename WeightMap::value_type,
                      std::list<typename Graph::edge_descriptor>>>
  yen_ksp(const Graph& g, Vertex<Graph> s, Vertex<Graph> t,
          WeightMap wm, IndexMap im, optional<unsigned> K)
  {
    using kr_type = Result<typename WeightMap::value_type, Graph>;

    // The shortest paths - these we return.
    std::list<kr_type> A;

    // An empty result if the source and destination are the same.
    if (s != t)
      {
        // The tentative paths - these are candidate paths.  It's a
        // set, because we want to make sure that a given result can
        // show up in the set of tentative results only once.  The
        // problem is that the algorithm can find the same tentative
        // path many times.
        std::set<kr_type> B;

        // In each iteration we produce the k-th shortest path.
        for (int k = 1; !K || k <= K.get(); ++k)
          if (!yen_ksp(g, s, t, wm, im, A, B))
            // We break the loop if no path was found.
            break;
      }

    return A;
  }

  template <typename Graph>
  std::list<std::pair<typename property_map<Graph, edge_weight_t>::value_type,
                      std::list<Edge<Graph>>>>
  yen_ksp(const Graph& g, Vertex<Graph> s, Vertex<Graph> t,
          optional<unsigned> K = optional<unsigned>())
  {
    return yen_ksp(g, s, t, get(edge_weight_t(), g),
                   get(vertex_index_t(), g), K);
  }

} // boost

#endif /* BOOST_GRAPH_YEN_KSP */
