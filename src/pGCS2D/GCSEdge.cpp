#include <cassert>
#include <string>
#include <utility>

#include "fusion.h"

#include "GCSEdge.h"

//---------------------------------------------------------
// Constructors

using mosek::fusion::Domain;
using mosek::fusion::Model;
using mosek::fusion::Variable;


GCSEdge::GCSEdge(
  EdgeId id,
  std::string name,
  GCSVertex* u,
  GCSVertex* v,
  Model::t M,
  bool relaxation)
  : m_id(id),
    m_name(std::move(name)),
    m_u(u),
    m_v(v),
    m_model(M)
{
  assert(!m_name.empty());
  if (relaxation)
    m_phi = m_model->variable(phiName(), 1, Domain::inRange(0, 1));
  else
    m_phi = m_model->variable(phiName(), 1, Domain::binary());

  m_y = m_model->variable(yName(), m_u->dim());
  m_z = m_model->variable(zName(), m_v->dim());
}


//---------------------------------------------------------
// Utilities

std::pair<const std::string, Variable::t> GCSEdge::addCostVar()
{
  const std::string name{ellNamePrefix() + std::to_string(m_ell.size())};
  auto ell{m_model->variable(name, 1)};
  m_ell.push_back(ell);
  return (std::pair<const std::string, Variable::t> {name, ell});
}


const std::vector<std::string> GCSEdge::ellNames() const {
  const std::string prefix{ellNamePrefix()};
  std::vector<std::string> names(m_ell.size());
  for (int i{0}; i < m_ell.size(); i++)
    names[i] = prefix + std::to_string(i);
  return (names);
}
