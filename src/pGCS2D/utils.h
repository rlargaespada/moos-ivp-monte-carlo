#include <Eigen/Dense>
#include <memory>

#include "fusion.h"

std::shared_ptr<monty::ndarray<double, 2>> toMosekArray(const Eigen::MatrixXi& M);

std::shared_ptr<monty::ndarray<double, 2>> toMosekArray(const Eigen::MatrixXd& M);
