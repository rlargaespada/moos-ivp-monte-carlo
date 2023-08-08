#ifndef GCS2D_CONVEX_SETS_HEADER
#define GCS2D_CONVEX_SETS_HEADER

//* system headers
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <utility>

//* external dependencies
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
    mosek::fusion::Variable::t x,
    const std::string& name) const = 0;
  virtual void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x,
    const std::string& name) const = 0;
  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    const Eigen::MatrixXd& A_x,
    const Eigen::VectorXd& b_x,
    const Eigen::VectorXd& c,
    double d,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const = 0;
  std::unique_ptr<ConvexSet> clone() const {return (makeClone());}

 protected:
  virtual std::unique_ptr<ConvexSet> makeClone() const = 0;
};

namespace ConvexSets {

class PointSet : public ConvexSet
{
 public:
  explicit PointSet(const XYPoint& pt) : m_pt(pt), m_x(pt.x(), pt.y()) {}

 public:
  virtual int dim() const {return (2);}  // points are in 2D
  const XYPoint& point() const {return (m_pt);}

  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const;
  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    const Eigen::MatrixXd& A_x,
    const Eigen::VectorXd& b_x,
    const Eigen::VectorXd& c,
    double d,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const;
  virtual void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x,
    const std::string& name) const;

 protected:
  virtual std::unique_ptr<ConvexSet> makeClone() const {
    return std::unique_ptr<PointSet> (new PointSet(*this));}

 private:
  const XYPoint m_pt;
  const Eigen::Vector2d m_x;
};


class PolyhedronSet : public ConvexSet
{
 public:
  explicit PolyhedronSet(const XYPolygon& poly, int power = 1);

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

  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const;
  virtual void addPerspectiveConstraint(
    mosek::fusion::Model::t M,
    const Eigen::MatrixXd& A_x,
    const Eigen::VectorXd& b_x,
    const Eigen::VectorXd& c,
    double d,
    mosek::fusion::Variable::t scale,
    mosek::fusion::Variable::t x,
    const std::string& name) const;
  virtual void addMembershipConstraint(
    mosek::fusion::Model::t M,
    mosek::fusion::Variable::t x,
    const std::string& name) const;

  // todo: requres cddlib to join XYPolys
  // PolyhedronSet cartesianProduct(const PolyhedronSet& other);
  // PolyhedronSet cartesianPower(int power) const;

 protected:
  virtual std::unique_ptr<ConvexSet> makeClone() const {
    return std::unique_ptr<PolyhedronSet> (new PolyhedronSet(*this));}

 private:
  void fromXYPolygon(const XYPolygon &poly);
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> getCartesianPower(int n);
  const XYPolygon m_poly;

  Eigen::MatrixXd m_A;
  Eigen::VectorXd m_b;
};
}  // namespace ConvexSets

#endif  // GCS2D_CONVEX_SETS_HEADER
