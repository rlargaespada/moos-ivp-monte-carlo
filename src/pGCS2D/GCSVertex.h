#ifndef GCS2D_GCSVERTEX_HEADER
#define GCS2D_GCSVERTEX_HEADER

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "convex_sets.h"


class GCSEdge;  // forward declaration
class GraphOfConvexSets;  // forward declaration


class GCSVertex
{
 public:
  const std::string& name() const {return (m_name);}
  int dim() const {return (m_set->dim());}
  const ConvexSet& set() const {return (*m_set);}
  const Eigen::VectorXd& x() const {return (m_x);}

  const std::vector<GCSEdge*>& incoming_edges() const {return (m_incoming_edges);}
  const std::vector<GCSEdge*>& outgoing_edges() const {return (m_outgoing_edges);}

  void addIncomingEdge(GCSEdge* e);
  void addOutgoingEdge(GCSEdge* e);

  void removeIncomingEdge(GCSEdge* e);
  void removeOutgoingEdge(GCSEdge* e);

 private:
  GCSVertex(std::string name, const ConvexSet& set);
  void setSolution();  // todo: what args?

  const std::string m_name;
  const std::unique_ptr<const ConvexSet> m_set;

  Eigen::VectorXd m_x;

  std::vector<GCSEdge*> m_incoming_edges;
  std::vector<GCSEdge*> m_outgoing_edges;

  // todo (future): ell, costs, constraints
};

#endif  // GCS2D_GCSVERTEX_HEADER