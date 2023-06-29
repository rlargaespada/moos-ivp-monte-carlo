#ifndef IRIS2D_IRISPROBLEM_HEADER
#define IRIS2D_IRISPROBLEM_HEADER

#include <Eigen/Dense>
#include <vector>
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISEllipse.h"
#include "IRISPolygon.h"


class IRISProblem
{
 public:
  IRISProblem(XYPoint seed, XYPolygon bounds);
  IRISProblem(XYPoint seed, IRISPolygon bounds);
  ~IRISProblem() {}

 public:
  bool run(int num_iters);

  void addObstacle(Eigen::Matrix2Xd obstacle) {m_obstacles.push_back(obstacle);}
  void addObstacle(XYPolygon obstacle);

  XYPolygon getPolygon() {return (m_polygon.toXYPolygon());}
  XYPolygon getEllipse(int num_pts = ELLIPSE_POLY_PTS) {return (m_ellipse.toXYPolygon(num_pts));}
  bool complete() {return (m_complete);}

 private:
  IRISPolygon m_polygon;
  IRISEllipse m_ellipse;

  std::vector<Eigen::Matrix2Xd> m_obstacles;
  IRISPolygon m_bounds;

  int m_iters_done;
  double m_best_volume;
  bool m_complete;

  //? other inputs: max iters, termination limit
};


#endif  // IRIS2D_IRISPROBLEM_HEADER
