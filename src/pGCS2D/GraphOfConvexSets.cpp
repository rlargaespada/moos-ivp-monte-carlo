#include <cassert>
#include <string>
#include <vector>

#include "fusion.h"

#include "XYPoint.h"
#include "XYPolygon.h"

#include "convex_sets.h"
#include "GCSVertex.h"
#include "GCSEdge.h"
#include "GraphOfConvexSets.h"


using mosek::fusion::Model;

//---------------------------------------------------------
// Constructors

int GraphOfConvexSets::s_vertex_id{0};
int GraphOfConvexSets::s_edge_id{0};


// default constructor, empty graph, no model, invalid order and continuity
GraphOfConvexSets::GraphOfConvexSets()
  : m_dimension(-1),
    m_order(-1),
    m_continuity(-1),
    m_source(nullptr),
    m_target(nullptr),
    m_model(nullptr)
{}


GraphOfConvexSets::GraphOfConvexSets(
  const std::vector<XYPolygon>& regions,
  int order,
  int continuity,
  const XYPoint& source,
  const XYPoint& target,
  const GraphOfConvexSetsOptions& options)
  : m_dimension(2),  // we're always in 2D
    m_order(order),
    m_continuity(continuity),
    m_source(nullptr),
    m_target(nullptr),
    m_options(options)
{
  assert(m_order > 0);
  assert(m_continuity >= 0);
  assert(m_continuity < m_order);

  m_model = new Model("gcs");

  // add a vertex for each input region
  for (const auto& region : regions)
    // add regions as cartesian products
    addVertex(ConvexSets::PolyhedronSet{region, m_order + 1}, region.get_label());

  // find edges between regions and add edges to graph
  const std::vector<std::pair<int, int>> edges_between_regions{findEdges(regions)};
  std::vector<GCSVertex*> vertex_vector{vertices()};
  for (auto e : edges_between_regions) {
    GCSVertex* u{vertex_vector.at(e.first)};
    GCSVertex* v{vertex_vector.at(e.second)};
    addEdge(u, v, u->name() + "->" + v->name());
  }

  // add start and goal to graph
  GCSVertex* m_source = addVertex(ConvexSets::PointSet(source), "source");
  GCSVertex* m_target = addVertex(ConvexSets::PointSet(target), "target");

  // find edges from start/to goal and add edges
  auto start_goal_edges{findStartGoalEdges(regions, source, target)};
  // todo: handle case where there's no start/goal edges
  for (int i : start_goal_edges.first) {
    GCSVertex* v{vertex_vector.at(i)};
    addEdge(m_source, v, "source->" + v->name());
  }
  for (int i : start_goal_edges.second) {
    GCSVertex* u{vertex_vector.at(i)};
    addEdge(u, m_target, u->name() + "->target");
  }

  // print testing
  // for (const auto& v : m_vertices)
  //   std::cout << v.second->id() << ": " << v.second->name() << std::endl;
  // for (const auto& e : edges_between_regions)
  //   std::cout << e.first << "," << e.second << std::endl;
  // for (int i : start_goal_edges.first)
  //   std::cout << "start->" << vertex_vector.at(i)->name() << std::endl;
  // for (int i : start_goal_edges.second)
  //   std::cout << vertex_vector.at(i)->name() << "->target" << std::endl;
  // for (const auto& e : m_edges)
  //   std::cout << e.second->id() << ": " << e.second->name() << std::endl;
  // m_model->writeTaskStream("ptf", std::cout);
}


//---------------------------------------------------------
// Methods for Changing Graph

GCSVertex* GraphOfConvexSets::addVertex(const ConvexSet& set, std::string name)
{
  if (name.empty())
    name = std::to_string(m_vertices.size());
  name.insert(0, "v_");  // prepend names with v_

  VertexId id{getNewVertexId()};
  auto emplace_result{m_vertices.emplace(id, new GCSVertex(s_vertex_id, name, set))};
  if (emplace_result.second)
    return (emplace_result.first->second.get());
  else  // todo: raise a warning or assert here?
    return (nullptr);
}


std::vector<std::pair<int, int>> GraphOfConvexSets::findEdges(
  const std::vector<XYPolygon>& regions) const
{
  std::vector<std::pair<int, int>> edges_between_regions;
  for (size_t i = 0; i < regions.size(); ++i) {
    for (size_t j = i + 1; j < regions.size(); ++j) {
      if (regions[i].intersects(regions[j])) {
        // Regions are overlapping, add edge.
        edges_between_regions.emplace_back(i, j);
        edges_between_regions.emplace_back(j, i);
      }
    }
  }

  return (edges_between_regions);
}


GCSEdge* GraphOfConvexSets::addEdge(GCSVertex* u, GCSVertex* v, std::string name)
{
  assert(u != nullptr);
  assert(v != nullptr);

  if (name.empty())
    name = std::to_string(m_edges.size());
  name.insert(0, "e_");  // prepend names with e_

  EdgeId id{getNewEdgeId()};
  auto emplace_result{m_edges.emplace(id, new GCSEdge(
    id, name, u, v, m_model, m_options.convex_relaxation))};
  if (emplace_result.second) {
    GCSEdge* e{emplace_result.first->second.get()};
    u->addOutgoingEdge(e);
    v->addIncomingEdge(e);
    return (e);
  } else {  // todo: raise a warning or assert here?
    return (nullptr);
  }
}


std::pair<std::vector<int>, std::vector<int>> GraphOfConvexSets::findStartGoalEdges(
  const std::vector<XYPolygon>& regions,
  const XYPoint& start,
  const XYPoint& goal) const
{
  std::vector<int> start_edges, goal_edges;
  for (size_t i{0}; i < regions.size(); ++i) {
    if (regions[i].contains(start))
      start_edges.push_back(i);
    if (regions[i].contains(goal))
      goal_edges.push_back(i);
  }
  return std::pair<std::vector<int>, std::vector<int>> {start_edges, goal_edges};
}


void GraphOfConvexSets::removeVertex(GCSVertex* vertex)
{
  assert(vertex != nullptr);
  VertexId id{vertex->id()};
  assert(m_vertices.count(id) > 0);

  for (auto it{m_edges.begin()}; it != m_edges.end();) {
    if ((it->second->u().id() == id) ||
        (it->second->v().id() == id)) {
      // remove edge variables from model
      it->second->m_phi->remove();
      it->second->m_z->remove();
      it->second->m_z->remove();
      it->second->m_ell->remove();
      it = m_edges.erase(it);
    } else {
      ++it;
    }
  }
  m_vertices.erase(id);
}


void GraphOfConvexSets::removeEdge(GCSEdge* edge)
{
  assert(edge != nullptr);
  assert(m_edges.count(edge->id()) > 0);
  edge->u().removeOutgoingEdge(edge);
  edge->v().removeIncomingEdge(edge);
  edge->m_phi->remove();
  edge->m_z->remove();
  edge->m_z->remove();
  edge->m_ell->remove();
  m_edges.erase(edge->id());
}


//---------------------------------------------------------
// Accessors

std::vector<GCSVertex*> GraphOfConvexSets::vertices()
{
  std::vector<GCSVertex*> vertices;
  vertices.reserve(m_vertices.size());
  for (const auto& v : m_vertices) {
    vertices.push_back(v.second.get());
  }
  return vertices;
}


std::vector<const GCSVertex*> GraphOfConvexSets::vertices() const
{
  std::vector<const GCSVertex*> vertices;
  vertices.reserve(m_vertices.size());
  for (const auto& v : m_vertices) {
    vertices.push_back(v.second.get());
  }
  return vertices;
}


std::vector<GCSEdge*> GraphOfConvexSets::edges()
{
  std::vector<GCSEdge*> edges;
  edges.reserve(m_edges.size());
  for (const auto& e : m_edges) {
    edges.push_back(e.second.get());
  }
  return edges;
}


std::vector<const GCSEdge*> GraphOfConvexSets::edges() const
{
  std::vector<const GCSEdge*> edges;
  edges.reserve(m_edges.size());
  for (const auto& e : m_edges) {
    edges.push_back(e.second.get());
  }
  return edges;
}
