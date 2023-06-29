#ifndef IRIS2D_IRISELLIPSE_HEADER
#define IRIS2D_IRISELLIPSE_HEADER

#include <Eigen/Dense>
#include "XYPoint.h"
#include "XYPolygon.h"


const double ELLIPSE_C_EPSILON{1e-4};
const int ELLIPSE_POLY_PTS{50};


class IRISEllipse
{
 public:
  explicit IRISEllipse(XYPoint seed, double radius = ELLIPSE_C_EPSILON);
  explicit IRISEllipse(Eigen::Vector2d seed, double radius = ELLIPSE_C_EPSILON);
  IRISEllipse(
    Eigen::Matrix2d C = Eigen::Matrix2d::Identity(),
    Eigen::Vector2d d = Eigen::Vector2d::Zero()): m_C{C}, m_d{d} {}
  ~IRISEllipse() {}

 public:
  const Eigen::Matrix2d& getC() {return m_C;}
  void setC(const Eigen::Matrix2d &C) {m_C = C;}
  void setCEntry(int row, int col, double val) {m_C(row, col) = val;}
  const Eigen::Vector2d& getD() {return m_d;}
  void setDEntry(int idx, double val) {m_d(idx) = val;}
  void setD(const Eigen::Vector2d &d) {m_d = d;}

  XYPolygon toXYPolygon(int num_pts = ELLIPSE_POLY_PTS);
  // area is det(C) * (area of circle of radius 1) = det(C) * PI
  double area() {return (m_C.determinant() * M_PI);}

 private:
  void fromSeed(Eigen::Vector2d seed, double radius);

  Eigen::Matrix2d m_C;
  Eigen::Vector2d m_d;
};


#endif  // IRIS2D_IRISELLIPSE_HEADER
