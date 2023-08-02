#include <cassert>
#include <string>

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
    m_continuity(-1)
{
  m_model = nullptr;
}


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
    m_options(options)
{
  assert(m_order > 0);
  assert(m_continuity >= 0);
  assert(m_continuity < m_order);

  m_model = new Model("gcs");

  // save source, target, regions?

  for (const auto& region : regions)
    addVertex(region);

  // if edges not provided, find edges
  // add edges
  // find source and target edges
  // add source and target vertices and edges

  // print testing
  // for (const auto& v : m_vertices)
  //   std::cout << v.first << ": " << v.second->name() << std::endl;
  // m_model->writeTaskStream("ptf", std::cout);
}


//---------------------------------------------------------
// Methods for Changing Graph

GCSVertex* GraphOfConvexSets::addVertex(const XYPolygon& region)
{
  std::string name{region.get_label()};
  if (name.empty())
    name = std::to_string(m_vertices.size());
  name.insert(0, "v");  // prepend names with v

  ConvexSets::PolyhedronSet set{region, m_order};  // add vertices as cartesian products
  auto emplace_result{m_vertices.emplace(s_vertex_id, new GCSVertex(name, set))};
  if (emplace_result.second)
    s_vertex_id++;
  // todo: handle when emplace fails
  return (emplace_result.first->second.get());
}
