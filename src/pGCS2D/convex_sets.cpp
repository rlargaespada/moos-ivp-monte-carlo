#include <Eigen/Dense>
#include <vector>

#include "fusion.h"

#include "convex_sets.h"


namespace ConvexSets
{

using mosek::fusion::Domain;
using mosek::fusion::Expr;
using mosek::fusion::Model;
using mosek::fusion::Var;
using mosek::fusion::Variable;


//---------------------------------------------------------
// PointSet

void PointSet::addPerspectiveConstraint(
  Model::t M,
  Variable::t scale,
  Variable::t x,
  const std::string& name
) const
{
  int d{dim()};
  Eigen::MatrixXd Aeq(d, d + 1);
  Aeq.leftCols(d) = Eigen::MatrixXd::Identity(d, d);
  Aeq.col(d) = -m_x;

  // convert Eigen Matrix to MOSEK array
  std::vector<std::vector<double>> Aeq_vec(Aeq.rows());
  for (int i{0}; i < Aeq.rows(); i++) {
    Aeq_vec.at(i).resize(Aeq.cols());
    for (int j{0}; j < Aeq.cols(); j++)
      Aeq_vec.at(i).at(j) = Aeq(i, j);
  }
  auto Abar_mosek{monty::new_array_ptr<double>(Aeq_vec)};

  Variable::t vars{Var::vstack(x, scale)};
  M->constraint(name, Expr::mul(Abar_mosek, vars), Domain::equalsTo(0));
}


void PointSet::addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    const Eigen::MatrixXd& A_x,
    const Eigen::VectorXd& b_x,
    const Eigen::VectorXd& c,
    double d,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const
{
  // A*x+b == (c'*scale+d)*x_.
  Eigen::MatrixXd Aeq(dim(), x->getSize() + scale->getSize());
  Aeq.leftCols(x->getSize()) = A_x;
  Aeq.rightCols(scale->getSize()) = -m_x * c.transpose();

  // convert Eigen Matrix to MOSEK array
  std::vector<std::vector<double>> Aeq_vec(Aeq.rows());
  for (int i{0}; i < Aeq.rows(); i++) {
    Aeq_vec.at(i).resize(Aeq.cols());
    for (int j{0}; j < Aeq.cols(); j++)
      Aeq_vec.at(i).at(j) = Aeq(i, j);
  }
  auto Aeq_mosek{monty::new_array_ptr<double>(Aeq_vec)};

  // set up beq and convert to mosek form
  Eigen::VectorXd beq{b_x - d * m_x};
  std::vector<double> beq_vec{beq.data(), beq.data() + beq.size()};
  auto beq_mosek{monty::new_array_ptr<double>(beq_vec)};

  Variable::t vars{Var::vstack(x, scale)};
  M->constraint(name, Expr::mul(Aeq_mosek, vars), Domain::equalsTo(beq_mosek));
}


void PointSet::addMembershipConstraint(
  Model::t M,
  Variable::t x,
  const std::string& name
) const
{}  // todo


//---------------------------------------------------------
// PolyhedronSet Constructors

void PolyhedronSet::fromXYPolygon(const XYPolygon &poly)
{
  // todo (future): use cddlib to do this calculation
  // resize A and b so we have one constraint per vertex of polygon
  m_A.resize(poly.size(), 2);
  m_b.resize(poly.size());
  Eigen::Vector2d poly_center{poly.get_centroid_x(), poly.get_centroid_y()};

  // fill in A and b using lines formed by adjacent vertices of polygon
  for (int i{0}; i < poly.size(); i++) {
    // get pairs of vertices of polygon
    int next_i{i < poly.size() - 1 ? i + 1 : 0};  // last vertex loops back to first vertex
    double x1{poly.get_vx(i)}, y1{poly.get_vy(i)};
    double x2{poly.get_vx(next_i)}, y2{poly.get_vy(next_i)};

    Eigen::Vector2d pt1{x1, y1};

    // use points to form linear inequality: A(i, 0) * x + A(i, 1)* y <= b(i)
    m_A.row(i) << y2 - y1, x1 - x2;
    m_b(i) = m_A.row(i).dot(pt1);

    // if center of polygon is on wrong side of constraint, flip signs of A and b
    if (m_A.row(i).dot(poly_center) > m_b(i)) {
      m_A.row(i) *= -1;
      m_b(i) *= -1;
    }
  }
}


std::pair<Eigen::MatrixXd, Eigen::VectorXd> PolyhedronSet::getCartesianPower(int n)
{
  Eigen::MatrixXd A_power{Eigen::MatrixXd::Zero(n * m_A.rows(), n * m_A.cols())};
  for (int i{0}; i < n; ++i) {
    A_power.block(i * m_A.rows(), i * m_A.cols(), m_A.rows(), m_A.cols()) = m_A;
  }
  Eigen::VectorXd b_power{m_b.replicate(n, 1)};
  return std::pair<Eigen::MatrixXd, Eigen::VectorXd>{A_power, b_power};
}


PolyhedronSet::PolyhedronSet(const XYPolygon& poly, int power)
  : m_poly(poly)
{
  // todo: can definitely be more efficient here
  assert(poly.is_convex());  // todo: manage this case better
  fromXYPolygon(poly);
  if (power > 1) {
    auto A_b_powers{getCartesianPower(power)};
    m_A = A_b_powers.first;
    m_b = A_b_powers.second;
  }
}


//---------------------------------------------------------
// PolyhedronSet Methods

bool PolyhedronSet::intersects(const XYPolygon& other) const
{
  return (m_poly.intersects(other));
}

bool PolyhedronSet::contains(const Eigen::VectorXd &point) const
{
  return ((m_A * point - m_b).maxCoeff() <= 0);
}


bool PolyhedronSet::contains(const XYPoint &point) const
{
  return (m_poly.contains(point));
}


bool PolyhedronSet::contains(double x, double y) const
{
  return (m_poly.contains(x, y));
}


void PolyhedronSet::addPerspectiveConstraint(
  Model::t M,
  Variable::t scale,
  Variable::t x,
  const std::string& name
) const
{
  // A x <= t b, written as [A,-b][x;scale] <= 0
  Eigen::MatrixXd Abar(m_A.rows(), m_A.cols() + 1);
  Abar.leftCols(m_A.cols()) = m_A;
  Abar.col(m_A.cols()) = -m_b;

  // convert Eigen Matrix to MOSEK array
  std::vector<std::vector<double>> Abar_vec(Abar.rows());
  for (int i{0}; i < Abar.rows(); i++) {
    Abar_vec.at(i).resize(Abar.cols());
    for (int j{0}; j < Abar.cols(); j++)
      Abar_vec.at(i).at(j) = Abar(i, j);
  }
  auto Abar_mosek{monty::new_array_ptr<double>(Abar_vec)};

  Variable::t vars{Var::vstack(x, scale)};
  M->constraint(name, Expr::mul(Abar_mosek, vars), Domain::lessThan(0));
}


void PolyhedronSet::addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    const Eigen::MatrixXd& A_x,
    const Eigen::VectorXd& b_x,
    const Eigen::VectorXd& c,
    double d,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const
{
  // A (A_x x + b_x) ≤ (c' scale + d) b, written as
  // [A * A_x, -b * c'][x;scale] <= d * b - A * b_x
  Eigen::MatrixXd Abar(m_A.rows(), x->getSize() + scale->getSize());
  Abar.leftCols(x->getSize()) = m_A * A_x;
  Abar.rightCols(scale->getSize()) = -m_b * c.transpose();

  // convert Eigen Matrix to MOSEK array
  std::vector<std::vector<double>> Abar_vec(Abar.rows());
  for (int i{0}; i < Abar.rows(); i++) {
    Abar_vec.at(i).resize(Abar.cols());
    for (int j{0}; j < Abar.cols(); j++)
      Abar_vec.at(i).at(j) = Abar(i, j);
  }
  auto Abar_mosek{monty::new_array_ptr<double>(Abar_vec)};

  // set up bbar and convert to mosek form
  Eigen::VectorXd bbar{d * m_b - m_A * b_x};
  std::vector<double> bbar_vec{bbar.data(), bbar.data() + bbar.size()};
  auto bbar_mosek{monty::new_array_ptr<double>(bbar_vec)};

  Variable::t vars{Var::vstack(x, scale)};
  M->constraint(name, Expr::mul(Abar_mosek, vars), Domain::lessThan(bbar_mosek));
}


void PolyhedronSet::addMembershipConstraint(
  Model::t M,
  Variable::t x,
  const std::string& name
) const
{}  // todo


}  // namespace ConvexSets
