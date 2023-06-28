#include <cdd/setoper.h>
#include <cdd/cdd.h>
#include <Eigen/Dense>
#include "XYPolygon.h"
#include "IRISPolygon.h"

//---------------------------------------------------------
// Constructors

IRISPolygon::IRISPolygon(Eigen::MatrixX2d A, Eigen::VectorXd b) : m_A{A}, m_b{b}
{
}


//---------------------------------------------------------
// Other Methods

void IRISPolygon::fromXYPolygon(const XYPolygon &poly)
{
  // resize A and b so we have one constraint per vertex of polygon
  m_A.resize(poly.size(), Eigen::NoChange);
  m_b.resize(poly.size());
  Eigen::Vector2d poly_center{poly.get_center_x(), poly.get_center_y()};

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


XYPolygon IRISPolygon::toXYPolygon()
{
  return XYPolygon{};
}


bool IRISPolygon::contains(const Eigen::Vector2d &point)
{
  return ((m_A * point - m_b).maxCoeff() <= 0);
}


bool IRISPolygon::contains(const XYPoint &point)
{
  return (contains(Eigen::Vector2d{point.x(), point.y()}));
}


bool IRISPolygon::contains(double x, double y)
{
  return (contains(Eigen::Vector2d{x, y}));
}


void IRISPolygon::appendConstraints(const IRISPolygon &other)
{
  m_A.conservativeResize(m_A.rows() + other.m_A.rows(), Eigen::NoChange);
  m_A.bottomRows(other.m_A.rows()) = other.getA();
  m_b.conservativeResize(m_b.rows() + other.m_b.rows());
  m_b.tail(other.m_b.rows()) = other.getB();
}


void IRISPolygon::appendConstraints(const XYPolygon &other)
{
  appendConstraints(IRISPolygon{other});
}
