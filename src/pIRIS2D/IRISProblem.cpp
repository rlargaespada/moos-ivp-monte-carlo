#include <Eigen/Dense>
#include <cmath>
#include "XYPoint.h"
#include "XYPolygon.h"
#include "IRISEllipse.h"
#include "IRISPolygon.h"
#include "iris_mosek.h"
#include "iris_ops.h"
#include "IRISProblem.h"


//---------------------------------------------------------
// Constructor()


IRISProblem::IRISProblem(int max_iters, double term_threshold)
{
  setup(XYPoint{0, 0}, IRISPolygon{}, max_iters, term_threshold);
}


IRISProblem::IRISProblem(XYPoint seed, XYPolygon bounds, int max_iters, double term_threshold)
{
  setup(seed, IRISPolygon{bounds}, max_iters, term_threshold);
}


IRISProblem::IRISProblem(XYPoint seed, IRISPolygon bounds, int max_iters, double term_threshold)
{
  setup(seed, bounds, max_iters, term_threshold);
}


void IRISProblem::setup(XYPoint seed, IRISPolygon bounds, int max_iters, double term_threshold)
{
  m_polygon = IRISPolygon{};  // empty to start
  m_ellipse = IRISEllipse{seed};
  m_bounds = bounds;

  m_iters_done = 0;
  m_best_area = m_ellipse.area();
  m_complete = false;

  m_max_iters = max_iters;
  m_termination_threshold = term_threshold;
}


//---------------------------------------------------------
// Procedure: run()

bool IRISProblem::run(int num_iters)
{
  double area;
  int iters{0};

  while (iters < num_iters) {
    // run iris iteration
    IRISPolygon new_poly{separating_lines(m_obstacles, m_ellipse)};
    new_poly.appendConstraints(m_bounds);
    m_polygon = new_poly;
    area = inner_ellipsoid(new_poly, &m_ellipse);

    // increment counters
    iters++;
    m_iters_done++;

    // check termination conditions
    if (((std::abs(area - m_best_area)/ m_best_area) < m_termination_threshold) ||
        (m_iters_done >= m_max_iters)) {
      m_complete = true;
      break;
    }

    // if we exceed num_iters without hitting other termination conditions,
    // IRIS hasn't converged yet and we need to continue later
    m_best_area = area;
    if (iters >= num_iters) {
      break;
    }
  }

  return (m_complete);
}


//---------------------------------------------------------
// Other Methods

void IRISProblem::addObstacle(XYPolygon obstacle)
{
  // pull out vertices of obstacle and place into a (2 x num_vertices) size matrix
  Eigen::Matrix2Xd obs_matrix(2, obstacle.size());
  for (int i{0}; i < obstacle.size(); i++)
    obs_matrix.col(i) << obstacle.get_vx(i), obstacle.get_vy(i);

  addObstacle(obs_matrix);
}
