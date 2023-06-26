#ifndef IRIS2D_IRISPROBLEM_HEADER
#define IRIS2D_IRISPROBLEM_HEADER

#include <Eigen/Dense>
#include <vector>
#include "XYOval.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISPolygon.h"
#include "IRISEllipse.h"


class IRISProblem
{
 public:
  IRISProblem(XYPoint seed, XYPolygon bounds);
  ~IRISProblem() {}

 public:
  void addObstacle(Eigen::Matrix2Xd obstacle) {m_obstacles.push_back(obstacle);}
  void addObstacle(XYPolygon obstacle);

  void run(unsigned int num_iters);

  XYPolygon getXYPolygon();
  XYOval getXYOval();

 private:
  IRISPolygon m_polygon;
  IRISEllipse m_ellipse;

  std::vector<Eigen::Matrix2Xd> m_obstacles;
  IRISPolygon m_bounds;

  int m_iters_done;
  double best_volume;
  bool m_complete;

  //? other inputs: max iters, termination limit
};


#endif  // IRIS2D_IRISPROBLEM_HEADER
