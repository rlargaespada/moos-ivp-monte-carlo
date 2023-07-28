#ifndef GCS2D_GCSEDGE_HEADER
#define GCS2D_GCSEDGE_HEADER

#include <string>
#include "GCSVertex.h"

class GCSEdge
{
 public:
  // todo: constructors

 public:
  const std::string& name() const { return m_name; }
  // const GCSVertex& const u() {return (m_u);}  // todo: make these modifiable
  // const GCSVertex& const v() {return (m_v);}
  // todo: xu, xv
  // todo: add costs, add constraints, add phi constraint, clear phi constraints
  // todo: get solution

 private:
  std::string m_name;
  GCSVertex m_u;
  GCSVertex m_v;

  // phi, y, z
  // allowed vars, x to yz
  // ell, costs, constraints
  // phi value
};

#endif  // GCS2D_GCSEDGE_HEADER
