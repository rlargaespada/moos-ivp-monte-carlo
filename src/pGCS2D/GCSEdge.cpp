#include <cassert>
#include <utility>

#include "fusion.h"

#include "GCSEdge.h"

//---------------------------------------------------------
// Constructors

using mosek::fusion::Model;
using mosek::fusion::Domain;

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
    m_v(v)
{
  assert(!m_name.empty());
  if (relaxation)
    m_phi = M->variable("phi_" + m_id, Domain::inRange(0, 1));
  else
    m_phi = M->variable("phi_" + m_id, Domain::binary());

  m_y = M->variable("y_" + m_id, m_u->dim());
  m_z = M->variable("z_" + m_id, m_v->dim());
  m_ell = M->variable("ell_" + m_id, 1);  // todo: is just one ok?
}
