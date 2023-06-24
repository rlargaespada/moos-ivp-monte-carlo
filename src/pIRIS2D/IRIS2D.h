/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IRIS2D.h                                          */
/*    DATE: June 14th, 2023                                 */
/************************************************************/

#ifndef IRIS2D_HEADER
#define IRIS2D_HEADER

#include <Eigen/Dense>
#include <map>
#include <set>
#include <string>
#include <queue>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPolygon.h"


class IRIS2D : public AppCastingMOOSApp
{
 public:
  IRIS2D();
  ~IRIS2D();

 protected:  // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:  // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();

  void handleRequests();
  bool handleObstacleAlert(std::string obs_alert);
  bool handleObstacleResolved(const std::string &obs_label);

  void syncObstacles();
  XYPoint randomSeedPoint();
  bool buildRegion(const XYPoint &seed);

 private:  // Configuration variables
  std::string m_obs_alert_var;  // OBSTACLE_ALERT
  std::string m_seed_pt_var;  // IRIS_SEED_POINT

  std::string m_iris_region_var;  // IRIS_REGION
  std::string m_complete_var;  // IRIS_COMPLETE

  // Visual params for rendering IRIS regions
  bool m_post_visuals;
  std::string m_label_color;
  std::string m_poly_fill_color;
  std::string m_poly_edge_color;
  std::string m_poly_vert_color;

  double m_poly_edge_size;
  double m_poly_vert_size;
  double m_poly_transparency;

  std::string m_ellipse_fill_color;
  std::string m_ellipse_edge_color;

  double m_ellipse_edge_size;
  double m_ellipse_transparency;

  // IRIS config
  std::string m_mode;
  unsigned int m_desired_regions;
  XYPolygon m_iris_bounds;
  unsigned int m_max_iters;
  double m_termination_threshold;

 private:  // State variables
  bool m_clear_pending;
  bool m_run_pending;
  bool m_iris_active;

  std::map<std::string, XYPolygon> m_obstacle_map;
  std::map<std::string, XYPolygon> m_obstacle_add_queue;
  std::set<std::string> m_obstacle_remove_queue;

  std::queue<XYPoint> m_seed_pt_queue;
};

#endif
