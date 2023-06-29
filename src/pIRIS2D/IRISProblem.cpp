#include <Eigen/Dense>
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISEllipse.h"
#include "IRISPolygon.h"
#include "IRISProblem.h"


//---------------------------------------------------------
// Constructor()

IRISProblem::IRISProblem(XYPoint seed, XYPolygon bounds)
{
  m_polygon = IRISPolygon{};  // empty to start
  m_ellipse = IRISEllipse{seed};
  m_bounds = IRISPolygon{bounds};

  m_iters_done = 0;
  m_best_volume = 0;
  m_complete = false;
}


IRISProblem::IRISProblem(XYPoint seed, IRISPolygon bounds)
{
  m_polygon = IRISPolygon{};  // empty to start
  m_ellipse = IRISEllipse{seed};
  m_bounds = bounds;

  m_iters_done = 0;
  m_best_volume = 0;
  m_complete = false;
}


//---------------------------------------------------------
// Procedure: run()

bool IRISProblem::run(int num_iters)
{
  return (true);
}


//---------------------------------------------------------
// Other Methods

void IRISProblem::addObstacle(XYPolygon obstacle)
{
  Eigen::Matrix2Xd obs_matrix(2, obstacle.size());
  for (int i{0}; i < obstacle.size(); i++)
    obs_matrix.col(i) << obstacle.get_vx(i), obstacle.get_vy(i);

  addObstacle(obs_matrix);
}
