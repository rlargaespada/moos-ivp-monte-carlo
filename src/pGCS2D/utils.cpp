#include <Eigen/Dense>
#include <cmath>
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


namespace  // unnamed namespace
{
std::vector<std::vector<int>> BINOMIAL_LUT {
  {1},                       // n = 0
  {1, 1},                    // n = 1
  {1, 2, 1},                 // n = 2
  {1, 3, 3, 1},              // n = 3
  {1, 4, 6, 4, 1},           // n = 4
  {1, 5, 10, 10, 5, 1},      // n = 5
  {1, 6, 15, 20, 15, 6, 1},  // n = 6
};

// todo: use more efficient method as used in GCS continuity constraints
void extendBinomialLUT(int n) {
  while (n >= BINOMIAL_LUT.size()) {
    size_t s{BINOMIAL_LUT.size()};
    std::vector<int> new_row(s + 1);
    new_row[0] = 1;

    for (int i{1}; i < s; i++)
      new_row[i] = BINOMIAL_LUT[s - 1][i - 1] + BINOMIAL_LUT[s - 1][i];
    new_row[s] = 1;

    BINOMIAL_LUT.emplace_back(new_row);
  }
}


int binomial(int n, int k) {
  extendBinomialLUT(n);
  return BINOMIAL_LUT[n][k];
}

}  // unnamed namespace

// todo: add optimizations for curves of order 2, 3
// todo: use matrix version of bezier curves: https://dergipark.org.tr/en/download/article-file/1633884
Eigen::VectorXd bezierSample(double t, const Eigen::MatrixXd& ctrl_pts)
{
  Eigen::Index dim{ctrl_pts.rows()}, order{ctrl_pts.cols() - 1};
  Eigen::VectorXd pt{Eigen::VectorXd::Zero(dim)};
  for (int k{0}; k <= order; k++)
    pt += ctrl_pts.col(k) * binomial(order, k) * std::pow(1 - t, order - k) * std::pow(t, k);
  return (pt);
}


Eigen::MatrixXd bezierSample(const Eigen::VectorXd& ts, const Eigen::MatrixXd& ctrl_pts)
{
  Eigen::MatrixXd pts{Eigen::MatrixXd::Zero(ctrl_pts.rows(), ts.size())};
  for (int p{0}; p < ts.size(); p++)
    pts.col(p) = bezierSample(ts(p), ctrl_pts);
  return (pts);
}
