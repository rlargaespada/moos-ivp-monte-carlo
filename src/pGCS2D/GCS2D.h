/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GCS2D.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GCS2D_HEADER
#define GCS2D_HEADER

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
  bool setEndpoints(std::string request);
  bool newIRISRegion(std::string spec);
  void handleNewWpt(int new_wpt_idx);

  // config handling
  bool readRegionsFromFile();

  // iterate loop helpers
  void requestIRISRegions();  // todo: handle timing so we clear IRIS and then run IRIS
  void buildGraph();
  void populateModel();
  bool checkPlanningPreconditions();  // todo: graph needs start and goal, edges from these/ model
  // todo: pull any other preconditions from gcs code
  bool planPath();
  void handlePlanningFail(std::string warning_msg = "");

  // path publishing
  std::string getPathStats();
  bool postPath();

  // state publishing
  std::string printPlannerMode();
  void postFlags(const std::vector<VarDataPair>& flags);

 private:  // Configuration variables
  // planning interface
  std::string m_path_request_var;  // default: PLAN_PATH_REQUESTED
  std::string m_wpt_complete_var;  // default: WAYPOINTS_COMPLETE

  // IRIS interface
  std::string m_iris_file;  // default: unset
  bool m_run_iris_on_new_path;  // default: true unless m_iris_file is set, else false
  // todo: handle cases where we get an iris file and this var, if we get a file this is ignored
  std::string m_run_iris_var;  // default: RUN_IRIS
  std::string m_clear_iris_var;  // default: CLEAR_IRIS
  std::string m_iris_region_var;  // default: IRIS_REGION
  std::string m_complete_var;  // default: IRIS_COMPLETE

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

  // GCS config
  bool m_use_bezier_gcs;
  GraphOfConvexSetsOptions m_options;

 private:  // Mode Enum
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

 private:  // State variables
  // start, goal, and vehicle positions as XYPoints
  XYPoint m_start_point;
  XYPoint m_goal_point;
  XYPoint m_vpos;

  // planning state data
  PlannerMode m_mode;
  double m_planning_start_time;
  double m_planning_end_time;
  double m_path_len_traversed;
  XYSegList m_path;

  // GCS state
  std::vector<XYPolygon> m_safe_regions;
  GraphOfConvexSets m_gcs;  // todo: if we have regions already, reuse existing graph edges
};

#endif
