#ifndef IRIS2D_IRISPROBLEM_HEADER
#define IRIS2D_IRISPROBLEM_HEADER

#include <Eigen/Dense>
#include <vector>
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISEllipse.h"
#include "IRISPolygon.h"


const int IRIS_DEFAULT_MAX_ITERS{100};
const double IRIS_DEFAULT_TERMINATION_THRESHOLD{2e-2};

class IRISProblem
{
 public:
  IRISProblem(
    int max_iters = IRIS_DEFAULT_MAX_ITERS,
    double term_threshold = IRIS_DEFAULT_TERMINATION_THRESHOLD);
  IRISProblem(
    XYPoint seed,
    XYPolygon bounds,
    int max_iters = IRIS_DEFAULT_MAX_ITERS,
    double term_threshold = IRIS_DEFAULT_TERMINATION_THRESHOLD);
  IRISProblem(
    XYPoint seed,
    IRISPolygon bounds,
    int max_iters = IRIS_DEFAULT_MAX_ITERS,
    double term_threshold = IRIS_DEFAULT_TERMINATION_THRESHOLD);
  ~IRISProblem() {}

 public:
  bool run(int num_iters);

  void addObstacle(Eigen::Matrix2Xd obstacle) {m_obstacles.push_back(obstacle);}
  void addObstacle(XYPolygon obstacle);

  IRISPolygon getIRISPolygon() {return (m_polygon);}
  IRISEllipse getIRISEllipse() {return (m_ellipse);}

  XYPolygon getPolygon() {return (m_polygon.toXYPolygon());}
  XYPolygon getEllipse(int num_pts = ELLIPSE_POLY_PTS) {return (m_ellipse.toXYPolygon(num_pts));}
  bool complete() {return (m_complete);}

 private:
  void setup(XYPoint seed, IRISPolygon bounds, int max_iters, double term_threshold);
  IRISPolygon m_polygon;
  IRISEllipse m_ellipse;

  std::vector<Eigen::Matrix2Xd> m_obstacles;
  IRISPolygon m_bounds;

  int m_iters_done;
  double m_best_area;
  bool m_complete;

  int m_max_iters;
  double m_termination_threshold;
};


#endif  // IRIS2D_IRISPROBLEM_HEADER
