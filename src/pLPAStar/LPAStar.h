/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: LPAStar.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef LPAStar_HEADER
#define LPAStar_HEADER

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "VarDataPair.h"
#include "XYConvexGrid.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "XYSegList.h"

enum class PlannerMode
{
  IDLE,
  REQUEST_PENDING,
  PLANNING_IN_PROGRESS,
  PLANNING_FAILED,  // todo: handle this case more thoughtfully
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
  bool handleObstacleResolved(std::string obs_label);

  // path planning
  bool checkPlanningPreconditions();  // LPA*, signature in base
  void syncObstacles();  // LPA*, signature in base
  bool planPath();  // LPA*, signature in base

  // LPA* utils
  std::set<int> getNeighbors(int grid_ix);
  double heuristic(int grid_ix);
  std::pair<double, double> calculateKey(int grid_ix);
  void initializeLPAStar();
  void updateVertex(int grid_ix);
  bool computeShortestPath();

  // path publishing
  std::string getPathStats();
  bool postPath();

  // replanning
  bool checkObstacles();
  bool replanFromCurrentPos();  // LPA*, signature in base

  // state publishing
  std::string printPlannerMode();
  void postFlags(const std::vector<VarDataPair>& flags);

 private:  // Configuration variables
  // vars to subscribe to
  std::string m_path_request_var;  // PLAN_PATH_REQUESTED
  std::string m_obs_alert_var;  // OBSTACLE_ALERT
  std::string m_wpt_complete_var;  // WAYPOINTS_COMPLETE

  // publication config
  std::string m_prefix;
  std::vector<VarDataPair> m_init_plan_flags;
  std::vector<VarDataPair> m_traverse_flags;
  std::vector<VarDataPair> m_replan_flags;
  std::vector<VarDataPair> m_end_flags;
  bool m_post_visuals;

  // grid config
  XYPolygon m_grid_bounds;
  XYConvexGrid m_grid;
  std::map<int, std::set<int>> m_neighbors;

  // planning config
  int m_max_iters;  // todo: add as config var, also higher level timeout?

  // not set in config
  std::string m_path_found_var;  // PATH_FOUND
  std::string m_path_complete_var;  // PATH_COMPLETE

 private:  // State variables
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  std::map<std::string, XYPolygon> m_obstacle_map;

  PlannerMode m_mode;
  double m_planning_start_time;
  double m_planning_end_time;

  XYSegList m_path;

  std::map<std::string, XYPolygon> m_obstacle_add_queue;
  std::set<std::string> m_obstacle_refresh_queue;
  std::set<std::string> m_obstacle_remove_queue;
};

#endif
