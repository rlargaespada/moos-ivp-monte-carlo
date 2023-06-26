#ifndef IRIS2D_IRIS_GEOM_HEADER
#define IRIS2D_IRIS_GEOM_HEADER

#include <Eigen/Dense>
#include "XYOval.h"
#include "XYPoint.h"
#include "XYPolygon.h"


const double ELLIPSOID_C_EPSILON = 1e-4;


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


class IRISEllipse
{
 public:
  explicit IRISEllipse(XYPoint seed);
  explicit IRISEllipse(Eigen::Vector2d seed);
  IRISEllipse(Eigen::Matrix2d C, Eigen::Vector2d d);
  ~IRISEllipse() {}

 public:
  const Eigen::Matrix2d& getC() {return m_C;}
  void setC(const Eigen::Matrix2d &C) {m_C = C;}
  void setCEntry(int row, int col, double val) {m_C(row, col) = val;}
  const Eigen::Vector2d& setD() {return m_d;}
  void setDEntry(int idx, double val) {m_d(idx) = val;}
  void setD(const Eigen::Vector2d &d) {m_d = d;}

  XYOval toXYOval();
  double volume();

 private:
  void fromSeed(Eigen::Vector2d seed);

  Eigen::Matrix2d m_C;
  Eigen::Vector2d m_d;
};


#endif  // IRIS2D_IRIS_GEOM_HEADER
