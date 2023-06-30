#include <Eigen/Dense>
#include <mosek.h>
#include <algorithm>
#include <numeric>
#include <vector>
#include "IRISEllipse.h"
#include "IRISPolygon.h"
#include "iris_mosek.h"
#include "iris_ops.h"


template <typename T>
std::vector<size_t> arg_sort(const std::vector<T> &vec) {
  std::vector<size_t> idx(vec.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&vec](size_t i0, size_t i1) {return vec[i0] < vec[i1];});
  return idx;
}


line tangent_line_through_point(
  const Eigen::Matrix2d &Cinv2,
  const Eigen::Vector2d &d,
  const Eigen::Vector2d &point)
{
  Eigen::Vector2d nhat = (2 * Cinv2  * (point - d)).normalized();
  return (line(nhat, nhat.transpose() * point));
}


IRISPolygon separating_lines(
  const std::vector<Eigen::Matrix2Xd> &obstacle_pts,
  const IRISEllipse &ellipse)
{
  size_t n_obs{obstacle_pts.size()};
  if (n_obs == 0)  // no obstacles, no constraints
    return (IRISPolygon{});

  // set up constants and loop vars
  Eigen::Matrix2d Cinv{ellipse.getC().inverse()};
  Eigen::Matrix2d Cinv2{Cinv * Cinv.transpose()};

  Eigen::Matrix<bool, Eigen::Dynamic, 1> uncovered_obstacles(n_obs, 1);
  uncovered_obstacles.setConstant(true);
  std::vector<line> lines;
  MSKenv_t env = NULL;

  // transform obstacles to ball space and sort by distance to ellipse seed
  std::vector<Eigen::Matrix2Xd> image_pts(n_obs);
  std::vector<Eigen::VectorXd> image_squared_dists(n_obs);
  std::vector<double> obs_min_squared_image_dists(n_obs);
  for (int i{0}; i < n_obs; i++) {
    image_pts[i] = Cinv * (obstacle_pts[i].colwise() - ellipse.getD());
    image_squared_dists[i] = image_pts[i].colwise().squaredNorm();
    obs_min_squared_image_dists[i] = image_squared_dists[i].minCoeff();
  }
  std::vector<size_t> obs_sort_idx = arg_sort(obs_min_squared_image_dists);

  for (size_t i : obs_sort_idx) {
    if (!uncovered_obstacles(i))
      continue;

    // find tangent line to ellipse through nearest vertex on nearest remaining obstacle
    int idx;
    image_squared_dists[i].minCoeff(&idx);
    Eigen::Vector2d closest_obs_vertex{obstacle_pts[i].col(idx)};
    line tangent_line{tangent_line_through_point(Cinv2, ellipse.getD(), closest_obs_vertex)};

    // calc distance from tangent line to obstacle
    Eigen::ArrayXXd dist_to_obs{(tangent_line.first.transpose() * obstacle_pts[i]).array()};
    dist_to_obs -= tangent_line.second;
    if ((dist_to_obs >= 0).all()) {
    // nhat already separates ellipse from obstacle i, so we can skip the optimization
      lines.push_back(tangent_line);
    } else {
      // otherwise, optimze to find the closest point on the obstacle to the ellipse
      if (!env)
        check_res(MSK_makeenv(&env, NULL));
      Eigen::Vector2d closest_pt{closest_point_in_convex_hull(image_pts[i], &env)};

      if (closest_pt.squaredNorm() < 1e-6) {
        // d is inside the obstacle. So we'll just reverse nhat to try to push the
        // ellipse out of the obstacle.
        lines.emplace_back(
          -tangent_line.first,
          -tangent_line.first.transpose() * obstacle_pts[i].col(idx));
      } else {
        Eigen::Vector2d xstar{ellipse.getC() * closest_pt + ellipse.getD()};
        lines.push_back(tangent_line_through_point(Cinv2, ellipse.getD(), xstar));
      }
    }

    // eliminate redundant obstacles
    line latest_tangent{lines.back()};
    for (size_t j{0}; j < n_obs; j++) {
      dist_to_obs = (latest_tangent.first.transpose() * obstacle_pts[j]).array();
      if ((dist_to_obs >= latest_tangent.second).all())
        uncovered_obstacles(j) = false;
    }
    uncovered_obstacles(i) = false;

    if (!uncovered_obstacles.any())
      break;
  }

  // construct A, b matrices and return polygon
  Eigen::MatrixX2d A(lines.size(), 2);
  Eigen::VectorXd b(lines.size());

  for (auto it = lines.begin(); it != lines.end(); ++it) {
    A.row(it - lines.begin()) = it->first.transpose();
    b(it - lines.begin()) = it->second;
  }

  return (IRISPolygon{A, b});
}
