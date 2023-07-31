#ifndef GCS2D_CONVEX_SETS_HEADER
#define GCS2D_CONVEX_SETS_HEADER

//* external dependencies
#include <Eigen/Dense>
#include "fusion.h"

//* local dependences
#include "XYPoint.h"
#include "XYPolygon.h"


class ConvexSet
{
 public:
  int dim();
  void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x);
  void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x);
 private:
};

namespace ConvexSets {

class PointSet : public ConvexSet
{
 public:
  int dim() {return (2);}  // points are in 2D
 private:
  XYPoint m_pt;
};


class PolygonSet : public ConvexSet
{
 public:
  explicit PolygonSet(const XYPolygon& poly);
 public:
  const Eigen::MatrixXd& A() const {return m_A;}
  const Eigen::VectorXd& b() const {return m_b;}
  int numConstraints() const {return (m_A.rows());}
  int dim() const {return (m_A.cols());}

  const XYPolygon& polygon() {return (m_poly);}
  const XYPolygon& polygon() const {return (m_poly);}
  XYPoint polyCentroid() {return (m_poly.get_centroid_pt());}

  bool intersects(const XYPolygon& other);
  // bool contains(const Eigen::VectorXd& point);
  bool contains(const XYPoint& point);
  bool contains(double x, double y);

  void addPerspectiveConstraint(  // todo: make sure vars are members of M
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x);
  void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x);

  PolygonSet cartesianProduct(const PolygonSet& other);
  PolygonSet cartesianPower(int power);

 private:
  XYPolygon m_poly;

  Eigen::MatrixXd m_A;  // todo: use Mosek matrices?
  Eigen::VectorXd m_b;
};
}  // namespace ConvexSets

#endif  // GCS2D_CONVEX_SETS_HEADER
