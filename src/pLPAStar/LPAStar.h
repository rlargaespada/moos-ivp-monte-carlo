/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: LPAStar.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef LPAStar_HEADER
#define LPAStar_HEADER

#include <map>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <XYPoint.h>
#include <XYPolygon.h>

class LPAStar : public AppCastingMOOSApp
{
 public:
  LPAStar();
  ~LPAStar();

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
  bool setEndpoints(std::string request);

  // path planning
  bool clearGrid();
  bool addObsToGrid();
  bool planPath();

  // path publishing
  double getPathLength();
  std::string getPathSpec();
  bool postPath();

  // replanning
  bool checkObstacles();
  bool replanFromCurrentPos();

 private:  // Configuration variables
  // todo: hard code any of these?
  std::string m_path_request_var;  // PLAN_PATH_REQUESTED (s)

  std::string m_obs_alert_var;  // OBSTACLE_ALERT (s)

  std::string m_path_found_var;  // PATH_FOUND (p)
  std::string m_wpt_update_var;  // PATH_UPDATE (p)
  std::string m_wpt_complete_var;  // WAYPOINTS_COMPLETE (s)
  std::string m_path_complete_var;  // PATH_COMPLETE (p)

  double m_grid_density;  // todo: think about how this would work
  XYPolygon m_grid_bounds;  // todo: add this
 private:  // State variables
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  std::map<std::string, XYPolygon> m_given_obstacles;
  std::map<std::string, XYPolygon> m_alerted_obstacles;  // todo: how long do obs stay in here?

  bool m_path_request_pending;
  bool m_transiting;
  bool m_replan_needed;

  std::vector<XYPoint> m_path;
};

#endif
