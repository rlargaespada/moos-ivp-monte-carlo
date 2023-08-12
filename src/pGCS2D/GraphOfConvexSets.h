#ifndef GCS2D_GRAPHOFCONVEXSETS_HEADER
#define GCS2D_GRAPHOFCONVEXSETS_HEADER

//* system headers
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

//* external dependencies
#include "fusion.h"

//* moos dependences
#include "XYPoint.h"
#include "XYPolygon.h"
#include "XYSegList.h"

//* local dependences
#include "GCSVertex.h"
#include "GCSEdge.h"


struct GraphOfConvexSetsOptions {
  bool convex_relaxation{true};
  unsigned int max_rounded_paths{10};
  bool preprocessing{true};
  unsigned int max_rounding_trials{100};
  double flow_tolerance{1e-5};
  int rounding_seed{0};

  // in C++11, using default member initializers prevents brace initialization
  // so add constructors to spawn these structs
  GraphOfConvexSetsOptions() {}  // empty constructor uses all defaults
};


class GraphOfConvexSets
{
 public:
  GraphOfConvexSets();
  GraphOfConvexSets(
    const std::vector<XYPolygon>& regions,
    int order,
    int continuity,
    const XYPoint& source,
    const XYPoint& target,
    const GraphOfConvexSetsOptions& options);  // actual constructor

  // todo: add optional existing edges to constructor
  // todo: constructor from existing graph to reuse vertices
  // todo: and edges with a new model (transfer ownership?)

  void dispose();
  ~GraphOfConvexSets() {dispose();}

 public:
  GCSVertex* addVertex(const ConvexSet& set, std::string name = "");
  GCSEdge* addEdge(GCSVertex* u, GCSVertex* v, std::string name = "");

  void removeVertex(GCSVertex* vertex);
  void removeEdge(GCSEdge* edge);

  int order() const {return (m_order);}
  int continuity() const{return (m_continuity);}
  int dimension() const {return (m_dimension);}
  std::vector<GCSVertex*> vertices();
  std::vector<const GCSVertex*> vertices() const;
  std::vector<GCSEdge*> edges();
  std::vector<const GCSEdge*> edges() const;
  const GCSVertex* const source() const {return (m_source);}
  const GCSVertex* const target() const {return (m_target);}
  const GraphOfConvexSetsOptions options() const {return (m_options);}
  const bool checkGraphOk() const {return (m_valid);}

  // todo: phi constraints

  void preprocessGraph();

  // todo: add exception handling on these operations
  void addContinuityConstraints();
  void addPathLengthCost(double weight);
  void addPathLengthCost(Eigen::MatrixXd weight_matrix);
  bool populateModel();

  void solveGCS();
  bool checkGCSDone();
  // todo: make accepted solution status an option, check problem status as well
  const bool checkSolverOk() const {
    return (m_model->getPrimalSolutionStatus() == mosek::fusion::SolutionStatus::Optimal);}
  const std::string errorMsg() const {return (m_mosek_error_msg);}

  void getRoundedPaths();
  bool relaxationRounding();

  void reconstructX();
  const XYSegList buildTrajectory(double step_size, double tolerance = 1e-3) const;

  // development and debugging
  void printModel() {m_model->writeTaskStream("ptf", std::cout);}

 private:
  VertexId getNewVertexId() {return (s_vertex_id++);}
  EdgeId getNewEdgeId() {return (s_edge_id++);}

  std::vector<std::pair<int, int>> findEdges(const std::vector<XYPolygon>& regions) const;
  std::pair<std::vector<int>, std::vector<int>> findStartGoalEdges(
    const std::vector<XYPolygon>& regions,
    const XYPoint& start,
    const XYPoint& goal) const;

  void addContinuityConstraint(int deriv, const Eigen::MatrixXi& Aeq);

  void addObjective();
  void addSpatialNonNegativityConstraints();

  void addVertexConstraints();
  void addConservationOfFlowConstraints(GCSVertex* v);
  void addDegreeConstraints(GCSVertex* v);

 private:
  static VertexId s_vertex_id;
  static EdgeId s_edge_id;

  bool m_valid;

  // curve params and options
  const int m_order;
  const int m_continuity;
  const int m_dimension;
  const int m_vertex_dim;
  const GraphOfConvexSetsOptions m_options;

  // graph members
  std::map<int, std::unique_ptr<GCSVertex>> m_vertices;
  std::map<int, std::unique_ptr<GCSEdge>> m_edges;
  GCSVertex* m_source;
  GCSVertex* m_target;

  // mosek optimization members
  mosek::fusion::Model::t m_model;
  std::thread m_mosek_thread;
  bool m_model_running;
  std::string m_mosek_error_msg;  // todo: use this somewhere

  // rounding members
  std::vector<std::vector<const GCSEdge*>> m_rounded_paths;
  mosek::fusion::Model::t m_best_rounded_model;
};

#endif  // GCS2D_GRAPHOFCONVEXSETS_HEADER
