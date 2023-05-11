/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DStarLite.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef DStarLite_HEADER
#define DStarLite_HEADER

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
  PLANNING_FAILED,
  IN_TRANSIT,
  PATH_COMPLETE,
};

typedef std::pair<double, double> dsl_key;  // keys in D* Lite consist of two doubles


class DStarLite : public AppCastingMOOSApp
{
 public:
  DStarLite();
  ~DStarLite();

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
  bool checkPlanningPreconditions();  // D* Lite, signature in base
  void syncObstacles();  // D* Lite, signature in base
  bool planPath();  // D* Lite, signature in base
  void handlePlanningFail(std::string warning_msg);

  // D* Lite utils
  int findCellByPoint(XYPoint pt);
  std::set<int> getNeighbors(int grid_ix);
  int getNextCell();

  double heuristic(int cell1, int cell2);
  double cost(int cell1, int cell2);
  dsl_key calculateKey(int grid_ix);
  void initializeDStarLite();
  void updateVertex(int grid_ix);

  bool computeShortestPath(int max_iters);
  XYSegList parsePathFromGrid();

  // path publishing
  std::string getPathStats();
  bool postPath();

  // replanning
  bool checkObstacles();

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

  // D* Lite config
  int m_max_iters;

  // not set in config
  std::string m_path_found_var;  // PATH_FOUND
  std::string m_path_complete_var;  // PATH_COMPLETE

 private:  // State variables
  // start, goal, and vehicle positions as XYPoints
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  // index of cell in graph that contains start, goal, and vehicle positions
  int m_start_cell;
  int m_goal_cell;

  // containers for obstacles we know about
  std::map<std::string, XYPolygon> m_obstacle_map;
  std::map<std::string, XYPolygon> m_obstacle_add_queue;
  std::set<std::string> m_obstacle_refresh_queue;
  std::set<std::string> m_obstacle_remove_queue;

  // planning state data
  PlannerMode m_mode;
  double m_planning_start_time;
  double m_planning_end_time;
  double m_path_len_traversed;
  XYSegList m_path;

  // D* Lite state
  int m_last_cell;  // last cell planned from, used for replanning
  double m_k_m;
  std::map<int, dsl_key> m_dstar_queue;
};

#endif
