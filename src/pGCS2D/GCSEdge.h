#ifndef GCS2D_GCSEDGE_HEADER
#define GCS2D_GCSEDGE_HEADER

#include <string>
#include <utility>
#include <vector>

#include "fusion.h"

#include "GCSVertex.h"


typedef int EdgeId;
class GraphOfConvexSets;  // forward declaration


class GCSEdge
{
 public:
  // accessors
  const VertexId id() const {return (m_id);}
  const GCSVertex& u() const {return (*m_u);}
  GCSVertex& u() {return (*m_u);}
  const GCSVertex& v() const {return (*m_v);}
  GCSVertex& v() {return (*m_v);}

  // string names
  const std::string strId() const {return (std::to_string(m_id));}
  const std::string& name() const {return m_name;}
  const std::string phiName() const {return ("phi_" + strId());}
  const std::string yName() const {return ("y_" + strId());}
  const std::string zName() const {return ("z_" + strId());}
  const std::vector<std::string> ellNames() const;

  const mosek::fusion::Variable::t& phi() const {return (m_phi);}
  // xu, xv?
  // todo: get solution, get solution phiXu, get soluton phiXv

  // todo: add constraint to model
  void addPhiConstraint(bool phi) {m_phi_constraint = phi ? 1 : 0;}
  void clearPhiConstraint() {m_phi_constraint = -1;}

 private:
  GCSEdge(
    EdgeId id,
    std::string name,
    GCSVertex* u,
    GCSVertex* v,
    mosek::fusion::Model::t M,
    bool relaxation = false);

  const std::string ellNamePrefix() const {return ("ell_" + strId() + "_");}
  std::pair<const std::string, mosek::fusion::Variable::t> addCostVar();

  const EdgeId m_id;
  const std::string m_name;
  GCSVertex* const m_u;
  GCSVertex* const m_v;

  mosek::fusion::Model::t m_model;
  mosek::fusion::Variable::t m_phi;
  mosek::fusion::Variable::t m_y;
  mosek::fusion::Variable::t m_z;
  std::vector<mosek::fusion::Variable::t> m_ell;

  int m_phi_constraint;

  friend class GraphOfConvexSets;

  // todo (future): add consts/constraints, allowed_vars, x_to_yz
};

#endif  // GCS2D_GCSEDGE_HEADER
