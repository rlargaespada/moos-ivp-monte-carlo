#ifndef IRIS2D_IRISPOLYGON_HEADER
#define IRIS2D_IRISPOLYGON_HEADER

#include <Eigen/Dense>
#include "XYPoint.h"
#include "XYPolygon.h"


class IRISPolygon
{
 public:
  IRISPolygon() {}  // default constructor is empty
  IRISPolygon(Eigen::MatrixX2d A, Eigen::VectorXd b): m_A{A}, m_b{b} {}
  explicit IRISPolygon(XYPolygon poly) {fromXYPolygon(poly);}
  ~IRISPolygon() {}

 public:
  const Eigen::MatrixX2d& getA() const {return m_A;}
  void setA(const Eigen::MatrixX2d &A) {m_A = A;}
  const Eigen::VectorXd& getB() const {return m_b;}
  void setB(const Eigen::VectorXd &b) {m_b = b;}
  int getNumConstraints() const {return (m_A.rows());}

  bool contains(const Eigen::Vector2d &point);
  bool contains(const XYPoint &point);
  bool contains(double x, double y);

  void appendConstraints(const IRISPolygon &other);
  void appendConstraints(const XYPolygon &other);

  XYPolygon toXYPolygon();

 private:
  void fromXYPolygon(const XYPolygon &poly);

  Eigen::MatrixX2d m_A;
  Eigen::VectorXd m_b;
};

#endif  // IRIS2D_IRISPOLYGON_HEADER
