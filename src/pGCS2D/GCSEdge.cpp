#include <cassert>
#include <utility>

#include "fusion.h"

#include "GCSEdge.h"

//---------------------------------------------------------
// Constructors

using mosek::fusion::Model;
using mosek::fusion::Domain;

GCSEdge::GCSEdge(std::string name, GCSVertex* u, GCSVertex* v, Model::t M, bool relaxation)
  : m_name(std::move(name)),
    m_u(u),
    m_v(v)
{
  assert(!m_name.empty());
  if (relaxation)
    m_phi = M->variable("phi_" + m_name, Domain::inRange(0, 1));
  else
    m_phi = M->variable("phi_" + m_name, Domain::binary());

  m_y = M->variable("y_" + m_name, m_u->dim());
  m_z = M->variable("z_" + m_name, m_v->dim());
  m_ell = M->variable("ell_" + m_name, 1);  // todo: is just one ok?
}
