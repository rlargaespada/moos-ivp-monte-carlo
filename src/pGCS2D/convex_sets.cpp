#include "convex_sets.h"


namespace ConvexSets
{

//---------------------------------------------------------
// PointSet

void PointSet::addPerspectiveConstraint(
  mosek::fusion::Model::t M,
  mosek::fusion::Variable::t scale,
  mosek::fusion::Variable::t x
) const
{}  // todo


void PointSet::addMembershipConstraint(
  mosek::fusion::Model::t M,
  mosek::fusion::Variable::t x
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
  mosek::fusion::Model::t M,
  mosek::fusion::Variable::t scale,
  mosek::fusion::Variable::t x
) const
{}  // todo


void PolyhedronSet::addMembershipConstraint(
  mosek::fusion::Model::t M,
  mosek::fusion::Variable::t x
) const
{}  // todo



}  // namespace ConvexSets
