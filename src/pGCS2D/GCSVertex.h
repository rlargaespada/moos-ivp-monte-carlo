#ifndef GCS2D_GCSVERTEX_HEADER
#define GCS2D_GCSVERTEX_HEADER

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "convex_sets.h"


typedef int VertexId;
class GCSEdge;  // forward declaration
class GraphOfConvexSets;  // forward declaration


class GCSVertex
{
 public:
  const VertexId id() const {return (m_id);}
  const std::string strId() const {return (std::to_string(m_id));}
  const std::string& name() const {return (m_name);}
  int dim() const {return (m_set->dim());}
  const ConvexSet& set() const {return (*m_set);}
  const Eigen::VectorXd& x() const {return (m_x);}

  const std::vector<GCSEdge*>& incoming_edges() const {return (m_incoming_edges);}
  const std::vector<GCSEdge*>& outgoing_edges() const {return (m_outgoing_edges);}

 private:
  GCSVertex(VertexId id, std::string name, const ConvexSet& set);
  void setSolution(const Eigen::VectorXd& sol);

  void addIncomingEdge(GCSEdge* e);
  void addOutgoingEdge(GCSEdge* e);
  void removeIncomingEdge(GCSEdge* e);
  void removeOutgoingEdge(GCSEdge* e);

  const VertexId m_id;
  const std::string m_name;
  const std::unique_ptr<const ConvexSet> m_set;
  Eigen::VectorXd m_x;

  std::vector<GCSEdge*> m_incoming_edges;
  std::vector<GCSEdge*> m_outgoing_edges;

  friend class GraphOfConvexSets;

  // todo (future): ell, add costs/constraints
};

#endif  // GCS2D_GCSVERTEX_HEADER
