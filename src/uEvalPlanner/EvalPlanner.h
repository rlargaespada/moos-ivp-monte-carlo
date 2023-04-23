/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.h                                   */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#ifndef EvalPlanner_HEADER
#define EvalPlanner_HEADER

#include <string>
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

  void clearPendingRequests();
  void clearMetrics();
  void clearTotalCounts();
  void clearTrialData();
  void initialize();

  bool handleResetSim();
  bool handleEndSim();

  bool handleResetTrial();
  bool handleSkipTrial();
  bool handleNextTrial();

  void calcMetrics();
  bool resetObstacles();
  bool resetVehicles();
  bool requestNewPath();

 private:  // Configuration variables
  XYPoint m_start_point;
  XYPoint m_goal_point;

  std::string m_path_request_var;
  std::string m_path_complete_var;

  std::string m_reset_sim_var;  // not set by config param
  std::string m_reset_obs_var;  // todo: needs to be a vector for each obs sim

  int m_desired_trials;

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
