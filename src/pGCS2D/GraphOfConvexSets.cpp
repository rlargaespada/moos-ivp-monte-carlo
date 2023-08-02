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

// default constructor, empty graph, no model, invalid order and continuity
GraphOfConvexSets::GraphOfConvexSets()
  : m_dimension(-1),
    m_order(-1),
    m_continuity(-1)
{
  m_model = nullptr;
}


GraphOfConvexSets::GraphOfConvexSets(
  const std::unordered_map<std::string, XYPolygon>& regions,
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
    addVertex(region.second, region.first);

  // if edges not provided, find edges
  // add edges
  // find source and target edges
  // add source and target vertices and edges

  // m_model->writeTaskStream("ptf", std::cout);
}


//---------------------------------------------------------
// Methods for Changing Graph

GCSVertex* GraphOfConvexSets::addVertex(const XYPolygon& region, std::string name)
{
  if (name.empty())
    name = "v" + std::to_string(m_vertices.size());

  // todo: make sure vertex name isn't in graph already, what would the effects be?
  // todo: handle cases where vertices are removed
  ConvexSets::PolyhedronSet set{region, m_order};  // add vertices as cartesian products
  auto emplace_result{m_vertices.emplace(name, new GCSVertex(name, set))};
  // todo: check emplace worked: emplece_result.second is a bool
  return (emplace_result.first->second.get());
}
