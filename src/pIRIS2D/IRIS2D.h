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

 private:  // Configuration variables
  std::string m_obs_alert_var;  // OBSTACLE_ALERT
  std::string m_seed_pt_var;  // IRIS_SEED_POINT

  std::string m_iris_region_var;  // IRIS_REGION
  std::string m_complete_var;
  bool m_post_visuals;

  std::string m_mode;
  unsigned int m_desired_regions;
  XYPolygon m_iris_bounds;
  unsigned int m_max_iters;
  // todo: add ellipsoid vol change threshold
  // todo: add visualization config params
 private:  // State variables
  bool m_clear_pending;
  bool m_run_pending;
  bool m_iris_active;

  // std::map<std::string,
  std::map<std::string, XYPolygon> m_obstacle_map;
  std::map<std::string, XYPolygon> m_obstacle_add_queue;
  std::set<std::string> m_obstacle_remove_queue;
};

#endif
