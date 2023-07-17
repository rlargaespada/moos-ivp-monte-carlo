/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NullPlanner.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef NullPlanner_HEADER
#define NullPlanner_HEADER

#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "VarDataPair.h"
#include "XYPolygon.h"
#include "XYSegList.h"


class NullPlanner : public AppCastingMOOSApp
{
 public:
  NullPlanner();
  ~NullPlanner();

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
  void handlePlanningFail(std::string warning_msg = "");

  // path publishing
  std::string getPathStats();
  bool postPath();

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
  std::vector<VarDataPair> m_end_flags;
  bool m_post_visuals;

  // publications variable names, not configurable by user
  std::string m_path_found_var;  // PATH_FOUND
  std::string m_path_complete_var;  // PATH_COMPLETE
  std::string m_path_stats_var;  // PATH_STATS
  std::string m_path_failed_var;  // PATH_FAILED

 private:  // Mode Enum
  enum class PlannerMode
  {
    IDLE,
    REQUEST_PENDING,
    PLANNING_IN_PROGRESS,
    PLANNING_FAILED,
    IN_TRANSIT,
    PATH_COMPLETE,
  };

 private:  // State variables
  // start, goal, and vehicle positions as XYPoints
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  // containers for obstacles we know about
  std::map<std::string, XYPolygon> m_obstacle_map;
  std::map<std::string, XYPolygon> m_obstacle_add_queue;
  std::set<std::string> m_obstacle_remove_queue;

  // planning state data
  PlannerMode m_mode;
  double m_planning_start_time;
  double m_planning_end_time;
  double m_path_len_traversed;
  XYSegList m_path;

  // nullplanner state
  XYSegList m_intermediate_pts;
  std::default_random_engine m_generator;
  std::uniform_real_distribution<double> m_perturbation{-1, 1};
};

#endif
