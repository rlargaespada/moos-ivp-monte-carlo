/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GCS2D.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GCS2D_HEADER
#define GCS2D_HEADER

#include <map>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "VarDataPair.h"
#include "XYSegList.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "GCSVertex.h"
#include "GCSEdge.h"
#include "GraphOfConvexSets.h"


class GCS2D : public AppCastingMOOSApp
{
 public:
  GCS2D();
  ~GCS2D();

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
  bool setEndpoints(const std::string& request);
  void handleNewWpt(int new_wpt_idx);
  void newIRISRegion(const std::string& spec);

  // config handling
  bool readRegionsFromFile();
  void checkConfigAssertions();

  // iterate loop helpers
  void clearIRISRegions();
  bool requestIRISRegions();
  bool buildGraph();
  bool populateModel();
  // todo: pull any other preconditions from gcs code
  bool checkPlanningPreconditions();  // todo: graph needs start and goal, edges from these/ model

  bool planPath();
  void handlePlanningFail(const std::string& warning_msg = "");

  // path publishing
  std::string getPathStats();
  bool postPath();

  // state publishing
  std::string printPlannerMode();
  std::string printGCSStep();
  void postFlags(const std::vector<VarDataPair>& flags);

 private:  // Configuration variables
  // planning interface
  std::string m_path_request_var;  // default: PLAN_PATH_REQUESTED
  std::string m_wpt_complete_var;  // default: WAYPOINTS_COMPLETE

  // IRIS interface
  std::string m_iris_file;  // default: unset
  bool m_run_iris_on_new_path;  // default: false
  // todo: handle cases where we get an iris file and this var, if we get a file this is ignored
  std::string m_run_iris_var;  // default: RUN_IRIS
  std::string m_clear_iris_var;  // default: CLEAR_IRIS
  std::string m_iris_region_var;  // default: IRIS_REGION
  std::string m_iris_complete_var;  // default: IRIS_COMPLETE

  // publication config
  std::string m_prefix;
  std::vector<VarDataPair> m_init_plan_flags;
  std::vector<VarDataPair> m_traverse_flags;
  std::vector<VarDataPair> m_end_flags;

  // publications variable names, not configurable by user
  std::string m_path_found_var;  // PATH_FOUND
  std::string m_path_complete_var;  // PATH_COMPLETE
  std::string m_path_stats_var;  // PATH_STATS
  std::string m_path_failed_var;  // PATH_FAILED

  // GCS config
  bool m_use_bezier_gcs;
  unsigned int m_bezier_order;
  unsigned int m_bezier_continuity_req;
  double m_path_length_weight;  // todo: should be able to apply separately in x and y
  // todo: should be able to apply these many times (use a vector? of pairs?)
  double m_derivative_regularization_weight;
  unsigned int m_derivative_regularization_order;
  GraphOfConvexSetsOptions m_options;

 private:  // State Enums
  enum class PlannerMode
  {
    IDLE,
    REQUEST_PENDING,
    IRIS_IN_PROGRESS,
    GCS_IN_PROGRESS,
    PLANNING_FAILED,
    IN_TRANSIT,
    PATH_COMPLETE,
  };

  enum class GCSStep
  {
    BUILD_GRAPH,
    PREPROCESS_GRAPH,
    POPULATE_MODEL,
    START_MOSEK,
    MOSEK_RUNNING,
    CONVEX_ROUNDING,
    FAILED,
  };

 private:  // State variables
  // start, goal, and vehicle positions as XYPoints
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  // planning state data
  PlannerMode m_mode;
  double m_planning_start_time;
  double m_planning_end_time;

  // path state data
  double m_path_len_traversed;
  int m_next_path_idx;
  XYSegList m_path;

  // GCS state
  std::map<std::string, XYPolygon> m_safe_regions;
  GraphOfConvexSets m_gcs;  // todo: if we have regions already, reuse existing graph edges
  GCSStep m_gcs_step;
};

#endif
