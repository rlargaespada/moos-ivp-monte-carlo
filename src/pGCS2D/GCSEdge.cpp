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
  std::string str_id{strId()};
  if (relaxation)
    m_phi = m_model->variable("phi_" + str_id, 1, Domain::inRange(0, 1));
  else
    m_phi = m_model->variable("phi_" + str_id, 1, Domain::binary());

  m_y = m_model->variable("y_" + str_id, m_u->dim());
  m_z = m_model->variable("z_" + str_id, m_v->dim());
}


//---------------------------------------------------------
// Utilities

std::pair<const std::string, Variable::t> GCSEdge::addCostVar()
{
  std::string name{"ell_" + strId() + "_" + std::to_string(m_ell.size())};
  auto ell{m_model->variable(name, 1)};
  m_ell.push_back(ell);
  return (std::pair<const std::string, Variable::t> {name, ell});
}
