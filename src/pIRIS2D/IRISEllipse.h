#ifndef IRIS2D_IRISELLIPSE_HEADER
#define IRIS2D_IRISELLIPSE_HEADER

#include <Eigen/Dense>
#include "XYPoint.h"
#include "XYOval.h"


const double ELLIPSOID_C_EPSILON = 1e-4;


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


#endif  // IRIS2D_IRISELLIPSE_HEADER
