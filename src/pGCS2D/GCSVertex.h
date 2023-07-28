#ifndef GCS2D_GCSVERTEX_HEADER
#define GCS2D_GCSVERTEX_HEADER

#include <Eigen/Dense>
#include <string>
#include "XYPoint.h"
#include "XYPolygon.h"


class GCSVertex
{
 public:
  // todo: constructors

 public:
  const std::string& name() const { return (m_name); }

  XYPolygon polygon() {return (m_polygon);}
  const XYPolygon& polygon() const {return (m_polygon);}
  XYPoint polyCentroid() {return (m_polygon.get_centroid_pt());}

  const Eigen::MatrixXd& A() const {return m_A;}
  // void setA(const Eigen::MatrixXd &A) {m_A = A;}
  const Eigen::VectorXd& b() const {return m_b;}
  // void setB(const Eigen::VectorXd &b) {m_b = b;}
  int numConstraints() const {return (m_A.rows());}
  int dim() const {return (m_A.cols());}


  bool intersects(const XYPolygon &other);
  bool contains(const Eigen::VectorXd &point);
  bool contains(const XYPoint &point);
  bool contains(double x, double y);

  GCSVertex translate();
  GCSVertex scale();
  // GCSVertex cartesianProduct(const GCSVertex &other);
  // GCSVertex cartesianPower(int n);

  void addPerspectiveConstraint();
  void addMembershipConstraint();
  // todo: additional costs and constraints
  // todo: get solution

  // todo: incoming and outgoing edges: getters, setters, removers
 private:
  void fillInMatrices(const XYPolygon &poly);

  std::string m_name;
  XYPolygon m_polygon;  // todo: how to handle point vertices?
  // todo: how to handle cartesian products?
  Eigen::MatrixXd m_A;  // todo: use Mosek matrices?
  Eigen::VectorXd m_b;

  // x, ell, costs, constraints
  // incoming and outgoing edges
};

#endif  // GCS2D_GCSVERTEX_HEADER
