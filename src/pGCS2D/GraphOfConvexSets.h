#ifndef GCS2D_GRAPHOFCONVEXSETS_HEADER
#define GCS2D_GRAPHOFCONVEXSETS_HEADER

//* system headers
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

//* external dependencies
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
#include "fusion.h"

//* moos dependences
#include "XYPoint.h"
#include "XYPolygon.h"

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
    const std::unordered_map<std::string, XYPolygon>& regions,
    int order,
    int continuity,
    const XYPoint& source,
    const XYPoint& target,
    const GraphOfConvexSetsOptions& options);  // actual constructor

  // todo: add optional existing edges to constructor
  // todo: constructor from existing graph to reuse vertices
  // todo: and edges with a new model (transfer ownership?)

  ~GraphOfConvexSets() {if (!(m_model == nullptr)) m_model->dispose();}

 public:
  // const std::vector<GCSEdge> incomingEdges(const std::string& name) const;
  // const std::vector<GCSEdge> incomingEdges(GCSVertex vertex) const;
  // const std::vector<GCSEdge> outgoingEdges(const std::string& name) const;
  // const std::vector<GCSEdge> outgoingEdges(GCSVertex vertex) const;
  // const std::vector<GCSEdge> incidentEdges(const std::string& name) const;
  // const std::vector<GCSEdge> incidentEdges(GCSVertex vertex) const;

  GCSVertex* addVertex(const XYPolygon& region, std::string name = "");
  void findEdges();
  void addEdge();

  void removeVertex(const std::string& name);
  void removeVertex(GCSVertex* vertex);
  void removeEdge();

  void vertices() const;
  void edges() const;

  // todo: phi constraints

  void preprocessGraph();
  bool populateModel();
  void solveGCS();
  void getSolutionPath();

 private:
  void addSourceTarget();
  void findStartGoalEdges();

  void addContinuityConstraints();

  void addPerspectiveCost();
  void addPerspectiveConstraint();

  void addConservationOfFlowConstraints();
  void addDegreeConstraints();
  void addCyclicConstraints();

  void relaxationRounding();

 private:
  const int m_order;
  const int m_continuity;
  const int m_dimension;

  std::map<std::string, std::unique_ptr<GCSVertex>> m_vertices;
  std::map<std::string, std::unique_ptr<GCSEdge>> m_edges;

  const GraphOfConvexSetsOptions m_options;
  mosek::fusion::Model::t m_model;
};

#endif  // GCS2D_GRAPHOFCONVEXSETS_HEADER
