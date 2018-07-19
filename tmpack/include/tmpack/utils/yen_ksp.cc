#define BOOST_TEST_MODULE yen_ksp

#include "yen_ksp.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <iostream>
#include <list>

// *******************************************************************
// Generic definitions.
// *******************************************************************

template <typename G>
using Edge = typename G::edge_descriptor;

template <typename G>
using Vertex = typename G::vertex_descriptor;

template <typename G>
using Path = std::list<Edge<G>>;

template <typename G>
using GPath = std::pair<const G&, Path<G>>;

template <typename G, typename T>
using Result = std::pair<T, Path<G>>;

template <typename G, typename T>
using GResult = std::pair<const G &, Result<G, T>>;

template <typename G, typename T>
using Results = std::list<Result<G, T>>;

// Add a directed edge, test it, and set weight.
template<typename G, typename T>
Edge<G>
ade(G &g, Vertex<G> s, Vertex<G> d, T w)
{
  Edge<G> e;
  bool success;

  boost::tie(e, success) = boost::add_edge(s, d, g);
  assert(success);

  boost::get(boost::edge_weight, g, e) = w;

  return e;
}

// Add an undirected edge.
template<typename G, typename T>
std::pair<Edge<G>, Edge<G>>
aue(G &g, Vertex<G> s, Vertex<G> d, T w)
{
  return std::make_pair(ade(g, s, d, w), ade(g, d, s, w));
}

template<typename G>
std::ostream &
operator << (std::ostream &out, const GPath<G> &p)
{
  for(auto const &e: p.second)
    out << e;
  return out;
}  

template<typename G, typename T>
std::ostream &
operator << (std::ostream &out, const GResult<G, T> &gr)
{
  out << gr.second.first << ": "
      << GPath<G>(gr.first, gr.second.second);
  return out;
}  

// *******************************************************************
// Specialized definitions.
// *******************************************************************

typedef
boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                      boost::no_property,
                      boost::property<boost::edge_weight_t, int> >
graph;

typedef Edge<graph> edge;
typedef Vertex<graph> vertex;
typedef Path<graph> path;
typedef Result<graph, int> result;
typedef Results<graph, int> results;

typedef GResult<graph, int> gresult;

// Check whether there is a given result.
bool
check_result(const results &rs, const result &r)
{
  return std::find(rs.begin(), rs.end(), r) != rs.end();
}

// *******************************************************************
// TESTS.
// *******************************************************************

//       a
//      /|\
//     1 | 1   --1--
//    /  |  \ /     \
//   b   2   c       d
//    \  |  / \     /
//     2 | 1   --10--
//      \|/
//       e

BOOST_AUTO_TEST_CASE(yen_ksp_test1)
{
  graph g(5);

  // vertexes
  auto i = vertices(g).first;
  vertex a = *i++;
  vertex b = *i++;
  vertex c = *i++;
  vertex d = *i++;
  vertex e = *i;

  // edges
  edge ab, ba;
  edge ae, ea;
  edge ac, ca;
  edge be, eb;
  edge cd1, dc1;
  edge cd2, dc2;
  edge ce, ec;

  tie(ab, ba) = aue(g, a, b, 1);
  tie(ae, ea) = aue(g, a, e, 2);
  tie(ac, ca) = aue(g, a, c, 1);
  tie(be, eb) = aue(g, b, e, 2);
  tie(cd1, dc1) = aue(g, c, d, 1);
  tie(cd2, dc2) = aue(g, c, d, 10);
  tie(ce, ec) = aue(g, c, e, 1);

  // We are looking for 4 KSPs.
  {
    auto r = boost::yen_ksp(g, b, d, 4);
    BOOST_CHECK(r.size() == 4);

    // Ending with cd1.
    BOOST_CHECK(check_result(r, result(3, path{ba, ac, cd1})));
    BOOST_CHECK(check_result(r, result(4, path{be, ec, cd1})));
    BOOST_CHECK(check_result(r, result(5, path{ba, ae, ec, cd1})));
    BOOST_CHECK(check_result(r, result(6, path{be, ea, ac, cd1})));
  }

  // We are looking for all KSPs.  There should be 8.
  {
    auto r = boost::yen_ksp(g, b, d);
    BOOST_CHECK(r.size() == 8);

    // Ending with cd1.
    BOOST_CHECK(check_result(r, result(3, path{ba, ac, cd1})));
    BOOST_CHECK(check_result(r, result(4, path{be, ec, cd1})));
    BOOST_CHECK(check_result(r, result(5, path{ba, ae, ec, cd1})));
    BOOST_CHECK(check_result(r, result(6, path{be, ea, ac, cd1})));

    // Ending with cd2.
    BOOST_CHECK(check_result(r, result(12, path{ba, ac, cd2})));
    BOOST_CHECK(check_result(r, result(13, path{be, ec, cd2})));
    BOOST_CHECK(check_result(r, result(14, path{ba, ae, ec, cd2})));
    BOOST_CHECK(check_result(r, result(15, path{be, ea, ac, cd2})));
  }
}

// Here we make sure that some tentative path does not appear in the
// set of tentative paths more than once.
//
// We search for all shortest paths in this graph:
//
//  /-----4-----\
// a--1--b---1---c
//        \--2--/
//
// There should be only three shortest paths.

BOOST_AUTO_TEST_CASE(yen_ksp_test2)
{
  graph g(3);

  // Vertexes
  auto i = vertices(g).first;
  vertex a = *i++;
  vertex b = *i++;
  vertex c = *i++;

  // Edges
  edge ab = ade(g, a, b, 1);
  edge bc1 = ade(g, b, c, 1);
  edge bc2 = ade(g, b, c, 2);
  edge ac = ade(g, a, c, 4);

  auto r = boost::yen_ksp(g, a, c);
  BOOST_CHECK(r.size() == 3);
  BOOST_CHECK(check_result(r, result(2, path{ab, bc1})));
  BOOST_CHECK(check_result(r, result(3, path{ab, bc2})));
  BOOST_CHECK(check_result(r, result(4, path{ac})));
}
