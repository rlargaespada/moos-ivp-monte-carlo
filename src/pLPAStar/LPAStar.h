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
#include <XYSegList.h>
#include <XYPolygon.h>


enum class PlannerMode
{
  IDLE,
  REQUEST_PENDING,
  PLANNING_IN_PROGRESS,
  PLANNING_FAILED,
  IN_TRANSIT,
  PATH_COMPLETE,
};


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
  bool handleObstacleAlert(std::string obs_alert);

  // path planning
  void clearGrid();
  bool addObsToGrid();
  bool planPath();

  // path publishing
  std::string getPathStats();
  bool postPath();

  // replanning
  bool checkObstacles();
  bool replanFromCurrentPos();

  // other
  std::string printPlannerMode();

 private:  // Configuration variables
  // todo: hard code any of these?
  std::string m_path_request_var;  // PLAN_PATH_REQUESTED (s)

  std::string m_obs_alert_var;  // OBSTACLE_ALERT (s)

  std::string m_path_found_var;  // PATH_FOUND (p)
  std::string m_wpt_update_var;  // PATH_UPDATE (p)
  std::string m_wpt_complete_var;  // WAYPOINTS_COMPLETE (s)
  std::string m_path_complete_var;  // PATH_COMPLETE (p)

  int m_max_iters;  // todo: add as config var

  double m_grid_density;  // todo: think about how this would work
  XYPolygon m_grid_bounds;  // todo: add this
  bool m_post_visuals;  // todo: add as config var
 private:  // State variables
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  // todo: add a sub to a variable that clears these
  std::map<std::string, XYPolygon> m_obstacle_map;

  PlannerMode m_mode;
  double m_planning_start_time;
  double m_planning_end_time;

  XYSegList m_path;
};

#endif
