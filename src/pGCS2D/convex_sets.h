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
  virtual int dim() const = 0;
  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x) const = 0;
  virtual void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x) const = 0;
 private:
};

namespace ConvexSets {

class PointSet : public ConvexSet
{
 public:
  int dim() const {return (2);}  // points are in 2D
  const XYPoint& point() const {return (m_pt);}

  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x) const;
  virtual void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x) const;

 private:
  const XYPoint m_pt;
};


class PolyhedronSet : public ConvexSet
{
 public:
  explicit PolyhedronSet(const XYPolygon& poly);
  PolyhedronSet(const XYPolygon& poly, int power);

 public:
  const Eigen::MatrixXd& A() const {return m_A;}
  const Eigen::VectorXd& b() const {return m_b;}
  int numConstraints() const {return (m_A.rows());}
  int dim() const {return (m_A.cols());}

  const XYPolygon& polygon() const {return (m_poly);}
  XYPoint polyCentroid() const {return (m_poly.get_centroid_pt());}

  bool intersects(const XYPolygon& other) const;
  bool contains(const Eigen::VectorXd& point) const;
  bool contains(const XYPoint& point) const;
  bool contains(double x, double y) const;

  virtual void addPerspectiveConstraint(  // todo: make sure vars are members of M
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x) const;
  virtual void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x) const;

  // todo: requres cddlib to join XYPolys
  // PolyhedronSet cartesianProduct(const PolyhedronSet& other);
  PolyhedronSet cartesianPower(int power) const;

 private:
  const XYPolygon m_poly;

  Eigen::MatrixXd m_A;  // todo: use Mosek matrices?
  Eigen::VectorXd m_b;
};
}  // namespace ConvexSets

#endif  // GCS2D_CONVEX_SETS_HEADER