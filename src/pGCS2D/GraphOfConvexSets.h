#ifndef GCS2D_GRAPHOFCONVEXSETS_HEADER
#define GCS2D_GRAPHOFCONVEXSETS_HEADER

//* system headers
#include <map>
#include <memory>
#include <string>
#include <utility>
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
    const std::vector<XYPolygon>& regions,
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

  // todo: phi constraints

  void preprocessGraph();
  bool populateModel();
  void solveGCS();
  void getSolutionPath();

 private:
  VertexId getNewVertexId() {return (s_vertex_id++);}
  EdgeId getNewEdgeId() {return (s_edge_id++);}

  std::vector<std::pair<int, int>> findEdges(const std::vector<XYPolygon>& regions) const;
  std::pair<std::vector<int>, std::vector<int>> findStartGoalEdges(
    const std::vector<XYPolygon>& regions,
    const XYPoint& start,
    const XYPoint& goal) const;

  void addContinuityConstraints();

  void addPerspectiveCost();
  void addPerspectiveConstraint();

  void addConservationOfFlowConstraints();
  void addDegreeConstraints();
  void addCyclicConstraints();

  void relaxationRounding();

 private:
  static VertexId s_vertex_id;
  static EdgeId s_edge_id;

  const int m_order;
  const int m_continuity;
  const int m_dimension;

  std::map<int, std::unique_ptr<GCSVertex>> m_vertices;
  std::map<int, std::unique_ptr<GCSEdge>> m_edges;
  GCSVertex* m_source;
  GCSVertex* m_target;

  const GraphOfConvexSetsOptions m_options;
  mosek::fusion::Model::t m_model;
};

#endif  // GCS2D_GRAPHOFCONVEXSETS_HEADER
