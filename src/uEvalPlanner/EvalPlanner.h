/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.h                                   */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#ifndef EvalPlanner_HEADER
#define EvalPlanner_HEADER

#include <map>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"

class EvalPlanner : public AppCastingMOOSApp
{
 public:
  EvalPlanner();
  ~EvalPlanner();

 protected:  // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:  // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();

  // state updates
  void clearPendingRequests();
  void clearMetrics();
  void clearTotalCounts();
  void clearTrialData();
  void initialize();

  // config handling
  bool handleConfigResetVars(std::string var_names);
  bool handleConfigEndflag(std::string flag);
  bool setVPoint(XYPoint* point, std::string point_spec);

  // actions during iteration
  bool handleResetSim();
  bool handleEndSim();
  bool handleResetTrial();
  bool handleSkipTrial();
  bool handleNextTrial();

  void calcMetrics();
  bool resetObstacles();
  bool resetVehicles();
  bool requestNewPath();
  bool postEndflags();

 private:  // Configuration variables
  XYPoint m_start_point;
  XYPoint m_goal_point;

  std::string m_path_request_var;
  std::string m_path_complete_var;

  std::string m_reset_sim_var;  // not set by config param
  std::vector<std::string> m_reset_obs_vars;

  int m_desired_trials;

  // default variable names
  std::string m_reset_obs_default;
  std::string m_path_complete_default;

  std::map<std::string, std::string> m_endflags;
  // todo: add time limit for trials

 private:  // State variables
  bool m_sim_active;
  bool m_reset_sim_pending;
  bool m_end_sim_pending;

  bool m_reset_trial_pending;
  bool m_skip_trial_pending;
  bool m_next_trial_pending;

  int m_encounter_count;
  int m_near_miss_count;
  int m_collision_count;
  int m_encounter_count_trial;
  int m_near_miss_count_trial;
  int m_collision_count_trial;

  int m_completed_trials;
};

#endif
