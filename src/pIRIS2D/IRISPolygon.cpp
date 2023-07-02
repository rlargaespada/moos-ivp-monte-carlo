#include <cdd/setoper.h>
#include <cdd/cdd.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISPolygon.h"


void IRISPolygon::fromXYPolygon(const XYPolygon &poly)
{
  // todo: use cddlib to do this calculation
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


// uses cddlib to compute vertices of convex polygon given by Ax <= b
// based on getGenerators() function defined in:
// https://github.com/rdeits/iris-distro/blob/master/src/cxx/iris_cdd.h
// assumes dd_set_global_constants() has already been called
XYPolygon IRISPolygon::toXYPolygon()
{
  XYPolygon null_poly;

  // only works if A and b have the same number of rows
  if (m_A.rows() != m_b.rows())
    return (null_poly);

  Eigen::Index dim{m_A.cols()};  // problem is 2D

  // fill in matrix for H-representation of polygon (Ax <= b)
  dd_MatrixPtr hrep{dd_CreateMatrix(m_A.rows(), dim + 1)};
  for (int i{0}; i < m_A.rows(); i++) {
    dd_set_d(hrep->matrix[i][0], m_b(i));
    for (int j{0}; j < dim; j++) {
      dd_set_d(hrep->matrix[i][j+1], -m_A(i, j));
    }
  }
  hrep->representation = dd_Inequality;

  // convert to V-representation of polygon (vertices and rays)
  dd_ErrorType err;
  dd_PolyhedraPtr poly = dd_DDMatrix2Poly(hrep, &err);
  if (err != dd_NoError)  // return empty polygon if there was an error
    return (null_poly);

  // parse out vertices of V-representation and put into a vector of XYPoints
  // save avg x/y to save a point inside the poly
  dd_MatrixPtr generators = dd_CopyGenerators(poly);
  std::vector<XYPoint> poly_vertices;
  double avg_x{0}, avg_y{0};

  for (int i{0}; i < generators->rowsize; i++) {
    double x, y;
    if (dd_get_d(generators->matrix[i][0]) == 1) {  // if first col is 1, it's a vertex
      x = dd_get_d(generators->matrix[i][1]);  // x is 2nd col of matrix
      y = dd_get_d(generators->matrix[i][2]);  // y is 3rd col of matrix
      avg_x += x;
      avg_y += y;
      poly_vertices.push_back(XYPoint(x, y));
    }
  }
  XYPoint avg_pt{avg_x/poly_vertices.size(), avg_y/poly_vertices.size()};

  // free memory allocated by cdd
  dd_FreeMatrix(hrep);
  dd_FreeMatrix(generators);
  dd_FreePolyhedra(poly);

  // sort poly vertices so they're ordered counter clockwise, makes XYPolygon convex
  // define lambda to compare pairs of vertices by angle from average point inside poly
  auto comp_vertices{
    [&avg_pt](XYPoint p1, XYPoint p2)-> bool {
      double ang_p1{std::atan2(p1.y() - avg_pt.y(), p1.x() - avg_pt.x())};
      double ang_p2{std::atan2(p2.y() - avg_pt.y(), p2.x() - avg_pt.x())};
      return (ang_p1 < ang_p2);
    }
  };
  std::sort(poly_vertices.begin(), poly_vertices.end(), comp_vertices);

  // place sorted vertices into XYPolyon and return
  XYPolygon solved_poly;
  for (const XYPoint &p : poly_vertices)
    solved_poly.add_vertex(p.x(), p.y(), false);
  solved_poly.determine_convexity();

  return (solved_poly);
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
