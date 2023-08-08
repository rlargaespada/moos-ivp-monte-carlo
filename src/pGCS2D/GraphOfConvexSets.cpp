#include <Eigen/Dense>
#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fusion.h"

#include "XYPoint.h"
#include "XYPolygon.h"

#include "convex_sets.h"
#include "GCSVertex.h"
#include "GCSEdge.h"
#include "GraphOfConvexSets.h"


using mosek::fusion::Domain;
using mosek::fusion::Expr;
// using mosek::fusion::Matrix;
using mosek::fusion::Model;
using mosek::fusion::ObjectiveSense;
using mosek::fusion::Variable;
using mosek::fusion::Var;


//---------------------------------------------------------
// Constructors

int GraphOfConvexSets::s_vertex_id{0};
int GraphOfConvexSets::s_edge_id{0};


// default constructor, empty graph, no model, invalid order and continuity
GraphOfConvexSets::GraphOfConvexSets()
  : m_dimension(-1),
    m_order(-1),
    m_continuity(-1),
    m_vertex_dim(-1),
    m_source(nullptr),
    m_target(nullptr),
    m_model(nullptr)
{}


GraphOfConvexSets::GraphOfConvexSets(
  const std::vector<XYPolygon>& regions,
  int order,
  int continuity,
  const XYPoint& source,
  const XYPoint& target,
  const GraphOfConvexSetsOptions& options)
  : m_dimension(2),  // we're always in 2D
    m_order(order),
    m_continuity(continuity),
    m_vertex_dim(2 * (order + 1)),
    m_source(nullptr),
    m_target(nullptr),
    m_options(options)
{
  assert(m_order > 0);
  assert(m_continuity >= 0);
  assert(m_continuity < m_order);

  m_model = new Model("gcs");
  // todo: only use variable/constraint names when debugging

  // add a vertex for each input region
  for (const auto& region : regions)
    // add regions as cartesian power of region polygons
    addVertex(ConvexSets::PolyhedronSet{region, m_order + 1}, region.get_label());

  // find edges between regions and add edges to graph
  const std::vector<std::pair<int, int>> edges_between_regions{findEdges(regions)};
  std::vector<GCSVertex*> vertex_vector{vertices()};
  for (auto e : edges_between_regions) {
    GCSVertex* u{vertex_vector.at(e.first)};
    GCSVertex* v{vertex_vector.at(e.second)};
    addEdge(u, v, u->name() + "->" + v->name());
  }

  // add start and goal to graph
  m_source = addVertex(ConvexSets::PointSet(source), "source");
  m_target = addVertex(ConvexSets::PointSet(target), "target");

  // find edges from start/to goal and add edges
  auto start_goal_edges{findStartGoalEdges(regions, source, target)};
  // todo: handle case where there's no start/goal edges
  for (int i : start_goal_edges.first) {
    GCSVertex* v{vertex_vector.at(i)};
    addEdge(m_source, v, "source->" + v->name());
  }
  for (int i : start_goal_edges.second) {
    GCSVertex* u{vertex_vector.at(i)};
    addEdge(u, m_target, u->name() + "->target");
  }

  // print testing
  // for (const auto& v : m_vertices)
  //   std::cout << v.second->id() << ": " << v.second->name() << std::endl;
  // for (const auto& e : edges_between_regions)
  //   std::cout << e.first << "," << e.second << std::endl;
  // for (int i : start_goal_edges.first)
  //   std::cout << "start->" << vertex_vector.at(i)->name() << std::endl;
  // for (int i : start_goal_edges.second)
  //   std::cout << vertex_vector.at(i)->name() << "->target" << std::endl;
  // for (const auto& e : m_edges)
  //   std::cout << e.second->id() << ": " << e.second->name() << std::endl;
  // m_model->writeTaskStream("ptf", std::cout);
}


//---------------------------------------------------------
// Methods for Changing Graph

GCSVertex* GraphOfConvexSets::addVertex(const ConvexSet& set, std::string name)
{
  if (name.empty())
    name = std::to_string(m_vertices.size());
  name.insert(0, "v_");  // prepend names with v_

  VertexId id{getNewVertexId()};
  auto emplace_result{m_vertices.emplace(id, new GCSVertex(id, name, set))};
  if (emplace_result.second)
    return (emplace_result.first->second.get());
  else  // todo: raise a warning or assert here?
    return (nullptr);
}


std::vector<std::pair<int, int>> GraphOfConvexSets::findEdges(
  const std::vector<XYPolygon>& regions) const
{
  std::vector<std::pair<int, int>> edges_between_regions;
  for (size_t i = 0; i < regions.size(); ++i) {
    for (size_t j = i + 1; j < regions.size(); ++j) {
      if (regions[i].intersects(regions[j])) {
        // Regions are overlapping, add edge.
        edges_between_regions.emplace_back(i, j);
        edges_between_regions.emplace_back(j, i);
      }
    }
  }

  return (edges_between_regions);
}


GCSEdge* GraphOfConvexSets::addEdge(GCSVertex* u, GCSVertex* v, std::string name)
{
  assert(u != nullptr);
  assert(v != nullptr);

  if (name.empty())
    name = std::to_string(m_edges.size());
  name.insert(0, "e_");  // prepend names with e_

  EdgeId id{getNewEdgeId()};
  auto emplace_result{m_edges.emplace(id, new GCSEdge(
    id, name, u, v, m_model, m_options.convex_relaxation))};
  if (emplace_result.second) {
    GCSEdge* e{emplace_result.first->second.get()};
    u->addOutgoingEdge(e);
    v->addIncomingEdge(e);
    return (e);
  } else {  // todo: raise a warning or assert here?
    return (nullptr);
  }
}


std::pair<std::vector<int>, std::vector<int>> GraphOfConvexSets::findStartGoalEdges(
  const std::vector<XYPolygon>& regions,
  const XYPoint& start,
  const XYPoint& goal) const
{
  std::vector<int> start_edges, goal_edges;
  for (size_t i{0}; i < regions.size(); ++i) {
    if (regions[i].contains(start))
      start_edges.push_back(i);
    if (regions[i].contains(goal))
      goal_edges.push_back(i);
  }
  return std::pair<std::vector<int>, std::vector<int>> {start_edges, goal_edges};
}


void GraphOfConvexSets::removeVertex(GCSVertex* vertex)
{
  assert(vertex != nullptr);
  VertexId id{vertex->id()};
  assert(m_vertices.count(id) > 0);

  for (auto it{m_edges.begin()}; it != m_edges.end();) {
    if ((it->second->u().id() == id) ||
        (it->second->v().id() == id)) {
      // remove edge variables from model
      it->second->m_phi->remove();
      it->second->m_z->remove();
      it->second->m_z->remove();

      for (auto ell : it->second->m_ell)
        ell->remove();  // remove from model

      it = m_edges.erase(it);
    } else {
      ++it;
    }
  }
  m_vertices.erase(id);
}


void GraphOfConvexSets::removeEdge(GCSEdge* edge)
{
  assert(edge != nullptr);
  assert(m_edges.count(edge->id()) > 0);
  edge->u().removeOutgoingEdge(edge);
  edge->v().removeIncomingEdge(edge);
  edge->m_phi->remove();
  edge->m_z->remove();
  edge->m_z->remove();
  for (auto ell : edge->m_ell)
    ell->remove();  // remove from model
  m_edges.erase(edge->id());
}


//---------------------------------------------------------
// Accessors

std::vector<GCSVertex*> GraphOfConvexSets::vertices()
{
  std::vector<GCSVertex*> vertices;
  vertices.reserve(m_vertices.size());
  for (const auto& v : m_vertices) {
    vertices.push_back(v.second.get());
  }
  return vertices;
}


std::vector<const GCSVertex*> GraphOfConvexSets::vertices() const
{
  std::vector<const GCSVertex*> vertices;
  vertices.reserve(m_vertices.size());
  for (const auto& v : m_vertices) {
    vertices.push_back(v.second.get());
  }
  return vertices;
}


std::vector<GCSEdge*> GraphOfConvexSets::edges()
{
  std::vector<GCSEdge*> edges;
  edges.reserve(m_edges.size());
  for (const auto& e : m_edges) {
    edges.push_back(e.second.get());
  }
  return edges;
}


std::vector<const GCSEdge*> GraphOfConvexSets::edges() const
{
  std::vector<const GCSEdge*> edges;
  edges.reserve(m_edges.size());
  for (const auto& e : m_edges) {
    edges.push_back(e.second.get());
  }
  return edges;
}


//---------------------------------------------------------
// Add Constraints and Costs

void GraphOfConvexSets::addContinuityConstraints()
{
  int scale_factor = 1;
  int dim{m_dimension};  // rneame to make typing easier

  /*
  Build up coefficient matrix for continuity constraints for each derivative.
  Matrix enforces that last control point of each curve u equals the first control
  point of the next curve v.

  From Bezier curve derivative laws: coefficient matrix matches rows of
  Pascal's triangle, scaled by curve order. Signs of each coefficient alternate.

  Examples:
    curve of order 3, derivative 1:
      (3!/(3 - 1)!) * [ 0  0  0  0  3  0 -3  0 -3  0  3  0  0  0  0  0 ]
      (3!/(3 - 1)!) * [ 0  0  0  0  0  3  0 -3  0 -3  0  3  0  0  0  0 ]
    curve of order 4, derivative 3:
      (4!/(4 - 3)!) * [ 0  0  1  0 -3  0  3  0 -1  0 -1  0  3  0 -3  0  1  0  0  0 ]
      (4!/(4 - 3)!) * [ 0  0  0  1  0 -3  0  3  0 -1  0 -1  0  3  0 -3  0  1  0  0 ]
  */

  for (int deriv{0}; deriv <= m_continuity; deriv++) {
    // matrices are zero to start
    Eigen::MatrixXi Aeq_u{Eigen::MatrixXi::Zero(dim, m_vertex_dim)};
    Eigen::MatrixXi Aeq_v{Eigen::MatrixXi::Zero(dim, m_vertex_dim)};

    int pascal{1}, num_coeff{deriv + 1};
    int sign{deriv % 2 == 0 ? 1 : -1};  // sign of coefficients alternates

    // calculate members of pascal's triangle, row (deriv + 1)
    Eigen::RowVectorXi pascal_coeffs(Eigen::RowVectorXi::Zero(num_coeff));
    for (int i{1}; i <= num_coeff; i++) {
      pascal_coeffs(i - 1) = (sign * pascal);
      pascal = pascal * (num_coeff - i) / i;  // next pascal coefficient
      sign *= -1;  // invert sign
    }

    // insert pascal coefficents into matrices
    for (int d{0}; d < dim; d++) {
      Aeq_u(d, Eigen::seqN(Eigen::last - dim + d + 1, num_coeff, -dim)) = pascal_coeffs.reverse();
      Aeq_v(d, Eigen::seqN(d, num_coeff, dim)) = pascal_coeffs;
    }

    // scale matrix by derivative
    Aeq_u *= scale_factor;
    Aeq_u *= -1;  // u matrix is negative, want v - u = 0
    Aeq_v *= scale_factor;
    scale_factor *= (m_order - deriv);

    Eigen::MatrixXi Aeq(m_dimension, 2 * m_vertex_dim);
    Aeq << Aeq_u, Aeq_v;

    // add continuity constraints to all edges except from source/to target
    addContinuityConstraint(deriv, Aeq);
  }

  // build source->vertex continuity constraint matrix
  Eigen::MatrixXi src_Aeq(Eigen::MatrixXi::Zero(dim, 1 + dim + m_vertex_dim));
  // first col is 0, next block is -I for source y, next block is I for vertex z
  src_Aeq.block(0, 1, dim, dim) = -Eigen::MatrixXi::Identity(dim, dim);
  src_Aeq.block(0, 1 + dim, dim, dim) = Eigen::MatrixXi::Identity(dim, dim);
  auto src_Aeq_mosek{toMosekArray(src_Aeq)};

  // add continuity constraints on source edges
  for (auto e : m_source->outgoing_edges()) {
    Variable::t phi_y_z{Var::vstack(e->m_phi, e->m_y, e->m_z)};
    std::string name{"e_" + e->strId() + "_continuity"};
    m_model->constraint(name, Expr::mul(src_Aeq_mosek, phi_y_z), Domain::equalsTo(0));
  }

  // build vertex->target continuity constraint matrix
  Eigen::MatrixXi targ_Aeq(Eigen::MatrixXi::Zero(dim, 1 + dim + m_vertex_dim));
  // first col is 0, next block is -I for vertex y, last block is I for target z
  targ_Aeq.block(0, 1 + m_vertex_dim - dim, dim, dim) = -Eigen::MatrixXi::Identity(dim, dim);
  targ_Aeq.rightCols(dim) = Eigen::MatrixXi::Identity(dim, dim);
  auto targ_Aeq_mosek{toMosekArray(targ_Aeq)};

  // add continuity constraints on target edges
  for (auto e : m_target->incoming_edges()) {
    Variable::t phi_y_z{Var::vstack(e->m_phi, e->m_y, e->m_z)};
    std::string name{"e_" + e->strId() + "_continuity"};
    m_model->constraint(name, Expr::mul(targ_Aeq_mosek, phi_y_z), Domain::equalsTo(0));
  }
}


void GraphOfConvexSets::addContinuityConstraint(int deriv, const Eigen::MatrixXi& Aeq)
{
  // todo: consider adding a dedicated Equality Perspective Constraint function
  // A = Aeq, b = Zeros
  // add extra column to Aeq for to build perspective constraint
    // A*x = b becomes A*x = phi*b
  Eigen::MatrixXi Aeq_ext(Aeq.rows(), Aeq.cols() + 1);
  Aeq_ext.col(0).setConstant(0);
  Aeq_ext.rightCols(Aeq.cols()) = Aeq;
  auto Aeq_mosek{toMosekArray(Aeq_ext)};

  // constrain all edges that aren't from source or to target
  std::vector<GCSEdge*> edge_vector{edges()};
  for (auto e : edge_vector) {
    if ((e->u().id() == m_source->id()) || (e->v().id() == m_target->id()))
      continue;
    Variable::t phi_y_z{Var::vstack(e->m_phi, e->m_y, e->m_z)};
    std::string name{"e_" + e->strId()+ "_continuity_" + std::to_string(deriv)};
    m_model->constraint(name, Expr::mul(Aeq_mosek, phi_y_z), Domain::equalsTo(0));
  }
}


void GraphOfConvexSets::addPathLengthCost(double weight)
{
  addPathLengthCost(weight * Eigen::MatrixXd::Identity(m_dimension, m_dimension));
}


void GraphOfConvexSets::addPathLengthCost(Eigen::MatrixXd weight_matrix)
{
  int dim{m_dimension};
  Eigen::MatrixXd A(dim, m_vertex_dim);
  Eigen::MatrixXd A_cone(dim + 1, m_vertex_dim + 2);
  std::vector<GCSEdge*> edge_vector{edges()};

  for (int i{0}; i < m_order; i++) {
    A.setConstant(0);
    A.block(0, i * dim, dim, dim) = -Eigen::MatrixXd::Identity(dim, dim);
    A.block(0, (i + 1) * dim, dim, dim) = Eigen::MatrixXd::Identity(dim, dim);
    A = weight_matrix * A;

    // todo: consider adding a dedicated add L2Norm Perspective Constraint method
    // A = A, b = zeros
    // set up matrix for quadratic cone constraint
    A_cone.setConstant(0);
    A_cone(0, 1) = 1;  // x_0 = ell
    A_cone.block(1, 0, A.rows(), 1) = Eigen::VectorXd::Zero(A.rows());  // b*phi
    A_cone.block(1, 2, A.rows(), A.cols()) = A;  // A * x_i
    auto A_cone_mosek{toMosekArray(A_cone)};

    for (auto e : edge_vector) {
      if (e->u().id() == m_source->id())
        continue;

      std::pair<std::string, Variable::t> ell{e->addCostVar()};  // add new cost to edge
      Variable::t phi_ell_y{Var::vstack(e->m_phi, ell.second, e->m_y)};
      std::string name{"e_" + e->strId() + "_path_cost_" + std::to_string(i)};
      m_model->constraint(name, Expr::mul(A_cone_mosek, phi_ell_y), Domain::inQCone());
    }
  }
}


//---------------------------------------------------------
// Populate Model

bool GraphOfConvexSets::populateModel()
{
  addObjective();
  // todo: handle phi constraints
  addSpatialNonNegativityConstraints();
  addVertexConstraints();
  return (true);
}


void GraphOfConvexSets::addObjective()
{
  // get a vector of all edge ells
  std::vector<Variable::t> ell_vec;
  for (auto& edge : m_edges)
    ell_vec.insert(ell_vec.end(), edge.second->m_ell.begin(), edge.second->m_ell.end());
  Variable::t ell_vars{Var::vstack(monty::new_array_ptr<Variable::t>(ell_vec))};

  // objective is to minimize all ells
  m_model->objective("min_cost", ObjectiveSense::Minimize, Expr::sum(ell_vars));
}


void GraphOfConvexSets::addSpatialNonNegativityConstraints()
{
  for (auto& edge : m_edges) {
    GCSEdge* e{edge.second.get()};
    std::string name_base{"e_" + e->strId() + "_spatial_nonneg_"};
    if (e->u().dim() > 0)
      e->u().set().addPerspectiveConstraint(m_model, e->m_phi, e->m_y, name_base + "_y");
    if (e->v().dim() > 0)
      e->v().set().addPerspectiveConstraint(m_model, e->m_phi, e->m_z, name_base + "_z");
  }
}


void GraphOfConvexSets::addVertexConstraints()
{
  for (auto& vertex : m_vertices) {
    GCSVertex* v{vertex.second.get()};
    addConservationOfFlowConstraints(v);
    addDegreeConstraints(v);
  }
}


// conservation of flow, spatial conservation of flow constraints
void GraphOfConvexSets::addConservationOfFlowConstraints(GCSVertex* v)
{
  bool is_src{v->id() == m_source->id()};
  bool is_targ{v->id() == m_target->id()};

  const std::vector<GCSEdge*>& incoming{v->incoming_edges()};
  const std::vector<GCSEdge*>& outgoing{v->outgoing_edges()};

  size_t degree{incoming.size() + outgoing.size()};
  if (degree == 0)  // no edges, don't add constraints
    return;

  // setup coefficient vector
  std::vector<double> a;
  a.insert(a.end(), incoming.size(), -1);
  a.insert(a.end(), outgoing.size(), 1);
  auto a_mosek{monty::new_array_ptr<double>(a)};

  // setup phis vector
  std::vector<Variable::t> vars(degree);
  int i{0};
  for (const GCSEdge* e : incoming)
    vars[i++] = e->m_phi;
  for (const GCSEdge* e : outgoing)
    vars[i++] = e->m_phi;
  auto phis_mosek{Var::vstack(monty::new_array_ptr<Variable::t>(vars))};

  // Conservation of flow: sum(phi_out) - sum(phi_in) = is_src - is_targ
  int phi_sum{(is_src ? 1 : 0) - (is_targ ? 1 : 0)};
  m_model->constraint("v_" + v->strId() + "_cons_flow",
    Expr::dot(a_mosek, phis_mosek), Domain::equalsTo(phi_sum));

  if ((!is_src) && (!is_targ)) {
    for (int d{0}; d < v->dim(); d++) {
      // setup zy vector
      i = 0;
      for (const GCSEdge* e : incoming)
        vars[i++] = e->m_z->index(d);
      for (const GCSEdge* e : outgoing)
        vars[i++] = e->m_y->index(d);
      auto zy_mosek{Var::vstack(monty::new_array_ptr<Variable::t>(vars))};

      // Spatial conservation of flow: sum(z_in) = sum(y_out)
      m_model->constraint("v_" + v->strId() + "_spat_cons_flow_" + std::to_string(d),
        Expr::dot(a_mosek, zy_mosek), Domain::equalsTo(0));
    }
  }
}


void GraphOfConvexSets::addDegreeConstraints(GCSVertex* v)
{
  bool is_src{v->id() == m_source->id()};
  bool is_targ{v->id() == m_target->id()};

  const std::vector<GCSEdge*>& incoming{v->incoming_edges()};
  const std::vector<GCSEdge*>& outgoing{v->outgoing_edges()};

  if (outgoing.empty())
    return;

  // create phi_out vector
  std::vector<Variable::t> phi_vec(outgoing.size());
  for (int i{0}; i < outgoing.size(); i++)
    phi_vec[i] = outgoing[i]->m_phi;
  auto phi_mosek{Var::vstack(monty::new_array_ptr<Variable::t>(phi_vec))};

  // Degree constraint: sum(phi_out) <= 1 - is_target
  m_model->constraint("v_" + v->strId() + "_degree",
    Expr::sum(phi_mosek), Domain::inRange(0, is_targ ? 0 : 1));

  // two cycle constraints
  if ((is_src) || (is_targ))
    return;

  // setup A_yz matrix, yz vector, a vector
  int vdim{v->dim()};
  // std::vector<double> a(outgoing.size(), 1);
  Eigen::VectorXd a(Eigen::VectorXd::Ones(outgoing.size()));
  std::vector<Variable::t> yz_vec(outgoing.size());
  Eigen::MatrixXd A_yz(vdim, outgoing.size() * vdim);
  for (int i{0}; i < outgoing.size(); i++) {
    yz_vec[i] = outgoing[i]->m_y;
    A_yz.block(0, i * vdim, vdim, vdim) = Eigen::MatrixXd::Identity(vdim, vdim);
  }

  for (int i{0}; i < outgoing.size(); i++) {
    const GCSEdge* e_out{outgoing[i]};
    if ((e_out->v().id() == m_source->id()) || (e_out->v().id() == m_target->id()))
      continue;

    for (const GCSEdge* e_in : incoming) {
      if (e_in->u().id() == e_out->v().id()) {
        a[i] = -1;
        phi_vec[i] = e_in->m_phi;

        // two-cycle constraint: sum(phi_u,out) - phi_uv - phi_vu >= 0
        std::vector<double> a_vec{a.data(), a.data() + a.size()};
        auto a_mosek{monty::new_array_ptr<double>(a_vec)};
        phi_mosek = Var::vstack(monty::new_array_ptr<Variable::t>(phi_vec));
        m_model->constraint("v_" + v->strId() + "_two_cycle_" + std::to_string(i),
          Expr::dot(a_mosek, phi_mosek), Domain::inRange(0, 1));

        A_yz.block(0, i * vdim, vdim, vdim) = -Eigen::MatrixXd::Identity(vdim, vdim);
        yz_vec[i] = e_in->m_z;

        // two-cycle spatial constraints
        auto yz_mosek{Var::vstack(monty::new_array_ptr<Variable::t>(yz_vec))};
        v->set().addPerspectiveConstraint(
          m_model,
          A_yz,
          Eigen::VectorXd::Zero(vdim),
          a,
          0,
          phi_mosek,
          yz_mosek,
          "v_" + v->strId() + "_spatial_two_cycle_" + std::to_string(i));

        // reset a, phi_vec, A_yz, yz_vec
        a[i] = 1;
        phi_vec[i] = e_out->m_phi;
        A_yz.block(0, i * vdim, vdim, vdim) = Eigen::MatrixXd::Identity(vdim, vdim);
        yz_vec[i] = e_out->m_y;
      }
    }
  }
}


//---------------------------------------------------------
// Solve model and extract path

void GraphOfConvexSets::solveGCS()
{
  m_model->solve();
}

//---------------------------------------------------------
// Utilities

// todo: put these in a utils file so they can be used in convex_sets.cpp

std::shared_ptr<monty::ndarray<double, 2>> GraphOfConvexSets::toMosekArray(const Eigen::MatrixXi& M)
{
  // first convert matrix to a vector of vectors so mosek can parse it
  std::vector<std::vector<double>> M_vec(M.rows());
  // todo: this can probably be more efficient
  for (int i{0}; i < M.rows(); i++) {
    M_vec.at(i).resize(M.cols());
    for (int j{0}; j < M.cols(); j++)
      M_vec.at(i).at(j) = M(i, j);
  }
  return (monty::new_array_ptr<double>(M_vec));
}


std::shared_ptr<monty::ndarray<double, 2>> GraphOfConvexSets::toMosekArray(const Eigen::MatrixXd& M)
{
  std::vector<std::vector<double>> M_vec(M.rows());
  // todo: this can probably be more efficient
  for (int i{0}; i < M.rows(); i++) {
    M_vec.at(i).resize(M.cols());
    for (int j{0}; j < M.cols(); j++)
      M_vec.at(i).at(j) = M(i, j);
  }
  return (monty::new_array_ptr<double>(M_vec));
}
