#ifndef GCS2D_GRAPHOFCONVEXSETS_HEADER
#define GCS2D_GRAPHOFCONVEXSETS_HEADER

#include <map>
#include <string>
#include <vector>

// workaround to use bezier library since both bezier and MOOS libraries define a PI constant
#ifdef PI
#define PI_TEMP PI
#undef PI
#endif
#include "bezier.h"

// undefine our temporary constant
#ifdef PI_TEMP
#define PI PI_TEMP
#undef PI_TEMP
#endif

#include "GCSVertex.h"
#include "GCSEdge.h"

struct GraphOfConvexSetsOptions {
  bool convex_relaxation{true};
  int max_rounded_paths{10};
  bool preprocessing{true};
  int max_rounding_trials{100};
  double flow_tolerance{1e-5};
  int rounding_seed{0};

  // in C++11, using default member initializers prevents brace initialization
  // so add constructors to spawn these structs
  GraphOfConvexSetsOptions() {}  // empty constructor uses all defaults
};


class GraphOfConvexSets
{
 public:
  // todo: constructor from regions
  // todo: constructor from regions + edges
  // todo: constructor from regions + edges + continuity + order (separate class?)
  // todo: constructor from existing graph to reuse vertices and edges with a new model
 public:
  std::vector<GCSEdge> incomingEdges(std::string name);
  std::vector<GCSEdge> incomingEdges(GCSVertex vertex);
  std::vector<GCSEdge> outgoingEdges(std::string name);
  std::vector<GCSEdge> outgoingEdges(GCSVertex vertex);
  std::vector<GCSEdge> incidentEdges(std::string name);
  std::vector<GCSEdge> incidentEdges(GCSVertex vertex);

  void addSourceTarget();
  void findStartGoalEdges();

  void addVertex();
  void findEdges();
  void addEdge();

  void removeVertex();
  void removeEdge();

  void vertices();
  void edges();

  // todo: phi constraints

  bool populateModel();
  void solveGCS();
  void getSolutionPath();

 private:
  // GCS helpers
  void preprocessGraph();

  void addPerspectiveCost();
  void addPerspectiveConstraint();

  void addConservationOfFlowConstraints();
  void addDegreeConstraints();
  void addCyclicConstraints();

  void relaxationRounding();

 private:
  std::map<std::string, GCSVertex> m_vertices;
  std::map<std::string, GCSEdge> m_edges;

  GraphOfConvexSetsOptions m_options;

  // todo: add model
  std::string m_status_msg;
};

#endif  // GCS2D_GRAPHOFCONVEXSETS_HEADER
