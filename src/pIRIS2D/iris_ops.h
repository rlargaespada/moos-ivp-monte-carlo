#include <Eigen/Dense>
#include <utility>
#include <vector>
#include "IRISEllipse.h"
#include "IRISPolygon.h"
#include "iris_mosek.h"


typedef std::pair<Eigen::Vector2d, double> line;

line tangent_line_through_point(
  const Eigen::Matrix2d &Cinv2,
  const Eigen::Vector2d &d,
  const Eigen::Vector2d &point);


IRISPolygon separating_lines(
  const std::vector<Eigen::Matrix2Xd> &obstacle_pts,
  const IRISEllipse &ellipse);

