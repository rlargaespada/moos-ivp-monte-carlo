#ifndef IRIS2D_IRIS_MOSEK_HEADER
#define IRIS2D_IRIS_MOSEK_HEADER

#include <mosek.h>
#include <exception>
#include <string>
#include "IRISEllipse.h"
#include "IRISPolygon.h"


class IRISMosekError : public std::exception {
 private:
  std::string message;
 public:
  explicit IRISMosekError(MSKrescodee res) {
    /* In case of an error print error code and description. */
    char symname[MSK_MAX_STR_LEN];
    char desc[MSK_MAX_STR_LEN];
    MSK_getcodedesc(res, symname, desc);
    message = std::string(symname) + ": " + std::string(desc);
  }

  const char * what() const throw() {
    return message.c_str();
  }
  ~IRISMosekError() throw() {}
};


class InnerEllipsoidInfeasibleError: public std::exception {
 public:
  const char * what() const throw() {
    return ("Inner ellipsoid problem is infeasible "
            "(this likely means that the polyhedron has no interior)");
  }
};


double inner_ellipsoid(
  const IRISPolygon &polygon,
  IRISEllipse *ellipse,
  MSKenv_t *existing_env = NULL);


Eigen::Vector2d closest_point_in_convex_hull(
  const Eigen::Matrix2Xd &Points,
  MSKenv_t *existing_env = NULL);


void check_res(MSKrescodee res);


#endif
