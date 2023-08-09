#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include <utility>

#include "convex_sets.h"

#include "GCSVertex.h"

//---------------------------------------------------------
// Constructors

GCSVertex::GCSVertex(VertexId id, std::string name, const ConvexSet& set)
  : m_id(id),
    m_name(std::move(name)),
    m_set(set.clone()) {
  assert(!m_name.empty());
  m_x.conservativeResize(m_set->dim());
  m_x.setConstant(std::nan(""));
}


//---------------------------------------------------------
// Edge Vectors

void GCSVertex::addIncomingEdge(GCSEdge* e)
{
  m_incoming_edges.push_back(e);
}


void GCSVertex::addOutgoingEdge(GCSEdge* e)
{
  m_outgoing_edges.push_back(e);
}


void GCSVertex::removeIncomingEdge(GCSEdge* e)
{
  auto i = std::remove(m_incoming_edges.begin(), m_incoming_edges.end(), e);
  m_incoming_edges.erase(i, m_incoming_edges.end());
}


void GCSVertex::removeOutgoingEdge(GCSEdge* e)
{
  auto i = std::remove(m_outgoing_edges.begin(), m_outgoing_edges.end(), e);
  m_outgoing_edges.erase(i, m_outgoing_edges.end());
}


//---------------------------------------------------------
// setSolution

void GCSVertex::setSolution(const Eigen::VectorXd& sol)
{
  assert(sol.size() == m_x.size());
  m_x = sol;
}
