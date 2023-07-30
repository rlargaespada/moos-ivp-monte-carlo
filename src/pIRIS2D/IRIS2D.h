/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IRIS2D.h                                          */
/*    DATE: June 14th, 2023                                 */
/************************************************************/

#ifndef IRIS2D_HEADER
#define IRIS2D_HEADER

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPolygon.h"
#include "IRISPolygon.h"
#include "IRISProblem.h"


struct IRISStats
{
  size_t idx;
  double duration;
  int num_iters;
  bool valid;
};


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

  // mail handling
  bool handleObstacleAlert(std::string obs_alert);
  bool handleObstacleResolved(const std::string &obs_label);

  // iterate loop helpers
  void handleRequests();
  void syncObstacles();
  XYPoint randomSeedPoint(XYPolygon container);
  XYPoint randomSeedPoint() {return randomSeedPoint(m_xy_iris_bounds);}
  bool checkFinishConditions();

  // iris methods
  bool seedOK(const XYPoint &seed);
  void setIRISProblem(const XYPoint &seed);
  bool runIRIS();
  bool saveIRISRegion(int idx = -1);

 private:  // Configuration variables
  std::string m_rebuild_iris_var;  // default: REBUILD_IRIS
  std::string m_run_iris_var;  // default: RUN_IRIS
  std::string m_clear_iris_var;  // default: CLEAR_IRIS

  std::string m_obs_alert_var;  // default: OBSTACLE_ALERT
  std::string m_seed_pt_var;  // default: IRIS_SEED_POINT

  std::string m_iris_region_var;  // default: IRIS_REGION
  std::string m_complete_var;  // default: IRIS_COMPLETE

  // Visual params for rendering IRIS regions
  std::string m_label_prefix;
  std::string m_label_color;

  bool m_post_poly_visuals;
  std::string m_poly_fill_color;
  std::string m_poly_edge_color;
  std::string m_poly_vert_color;
  double m_poly_edge_size;
  double m_poly_vert_size;
  double m_poly_transparency;

  bool m_post_ellipse_visuals;
  std::string m_ellipse_fill_color;
  std::string m_ellipse_edge_color;
  std::string m_ellipse_vert_color;
  double m_ellipse_edge_size;
  double m_ellipse_vert_size;
  double m_ellipse_transparency;

  // IRIS config
  std::string m_mode;
  unsigned int m_desired_regions;
  XYPolygon m_xy_iris_bounds;
  IRISPolygon m_iris_bounds;
  unsigned int m_max_iters;
  double m_termination_threshold;

 private:  // State variables
  // high level app state
  bool m_clear_pending;
  bool m_run_pending;
  bool m_active;

  // obstacle state
  std::map<std::string, XYPolygon> m_obstacle_map;
  std::map<std::string, XYPolygon> m_obstacle_add_queue;
  std::set<std::string> m_obstacle_remove_queue;

  // IRIS state
  std::deque<XYPoint> m_seed_pt_queue;
  IRISProblem m_current_problem;
  bool m_iris_in_progress;
  int m_iris_region_idx;
  double m_iris_start_time;

  // IRIS outputs
  std::vector<XYPolygon> m_safe_regions;
  std::vector<XYPolygon> m_iris_ellipses;
  std::set<int> m_invalid_regions;
  std::deque<IRISStats> m_iris_stats;
};

#endif
