#include <Eigen/Dense>
#include <cmath>
#include "AngleUtils.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISEllipse.h"


//---------------------------------------------------------
// Constructors

IRISEllipse::IRISEllipse(XYPoint seed, double radius)
{
  fromSeed(Eigen::Vector2d{seed.x(), seed.y()}, radius);
}


IRISEllipse::IRISEllipse(Eigen::Vector2d seed, double radius)
{
  fromSeed(seed, radius);
}


void IRISEllipse::fromSeed(Eigen::Vector2d seed, double radius)
{
  m_C = radius * Eigen::Matrix2d::Identity();
  m_d = seed;
}


//---------------------------------------------------------
// Other Methods

XYPolygon IRISEllipse::toXYPolygon(int num_pts)
{
  // get linspace between 0 and 2PI, add extra pt so first {num_pts} pts don't roll over
  Eigen::Array<double, 1, Eigen::Dynamic> ts;
  ts.setLinSpaced(num_pts + 1, 0, 2*M_PI);

  // vstack cos and sin of linspace and apply C and d
  Eigen::Matrix<double, 2, Eigen::Dynamic> points(2, ts.cols());
  points << ts.cos(), ts.sin();
  points = (m_C * points).colwise() + m_d;

  // form polygon out of elllipse points and return
  XYPolygon ellipse_poly;
  for (int i=0; i < num_pts ; i++) {
    ellipse_poly.add_vertex(points(0, i), points(1, i));
  }
  ellipse_poly.determine_convexity();
  return (ellipse_poly);
}
