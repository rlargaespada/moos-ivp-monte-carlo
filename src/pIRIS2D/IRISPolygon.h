#ifndef IRIS2D_IRISPOLYGON_HEADER
#define IRIS2D_IRISPOLYGON_HEADER

#include <Eigen/Dense>
#include "XYPoint.h"
#include "XYPolygon.h"


class IRISPolygon
{
 public:
  IRISPolygon();  //? what to do here?
  IRISPolygon(Eigen::MatrixX2d A, Eigen::VectorXd b);
  ~IRISPolygon() {}

 public:
  const Eigen::MatrixX2d& getA() {return m_A;}
  void setA(const Eigen::MatrixX2d &A) {m_A = A;}
  const Eigen::VectorXd& getB() {return m_b;}
  void setB(const Eigen::VectorXd &b) {m_b = b;}

  XYPolygon toXYPolygon();
  bool contains(const Eigen::Vector2d &point);
  bool contains(const XYPoint &point);  //? remove?
  void appendConstraints(const IRISPolygon &other);
  void appendConstraints(const XYPolygon &other);  //? remove?

 private:
  Eigen::MatrixX2d m_A;
  Eigen::VectorXd m_b;
  //? representation dirty?
  //? get generators? dd_check?
};

#endif  // IRIS2D_IRISPOLYGON_HEADER
