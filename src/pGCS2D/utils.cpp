#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "fusion.h"

#include "utils.h"


// todo: these can probably be more efficient
std::shared_ptr<monty::ndarray<double, 2>> toMosekArray(const Eigen::MatrixXi& M)
{
  // first convert matrix to a vector of vectors so mosek can parse it
  std::vector<std::vector<double>> M_vec(M.rows());
  for (int i{0}; i < M.rows(); i++) {
    M_vec.at(i).resize(M.cols());
    for (int j{0}; j < M.cols(); j++)
      M_vec.at(i).at(j) = M(i, j);
  }
  return (monty::new_array_ptr<double>(M_vec));
}

std::shared_ptr<monty::ndarray<double, 2>> toMosekArray(const Eigen::MatrixXd& M)
{
  std::vector<std::vector<double>> M_vec(M.rows());
  for (int i{0}; i < M.rows(); i++) {
    M_vec.at(i).resize(M.cols());
    for (int j{0}; j < M.cols(); j++)
      M_vec.at(i).at(j) = M(i, j);
  }
  return (monty::new_array_ptr<double>(M_vec));
}

// todo: add similar conversion for 1D vector
