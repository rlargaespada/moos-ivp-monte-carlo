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


struct TrialData
{
  int trial_num{0};
  bool trial_successful{true};

  int encounter_count{0};
  int near_miss_count{0};
  int collision_count{0};

  // in C++11, using default member initializers prevents brace initialization
  // so add constructors to spawn these structs
  TrialData() {}  // empty constructor uses all defaults
  explicit TrialData(int num) : trial_num{num} {};  // specify trial number
};


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

  // initialization routines
  void clearPendingRequests();
  void clearCurrentTrialData();
  void clearCurrentTrialData(int trial_num);
  void initialize();

  // config handling
  bool handleConfigResetVars(std::string var_names);
  bool handleConfigEndflag(std::string flag);
  bool setVPoint(XYPoint* point, std::string point_spec);
  bool setVPointConfig(XYPoint* point, std::string point_spec);

  // mail handling
  void handleSimRequest(std::string request, bool* pending_flag);

  // actions during iteration
  bool handleResetSim();
  bool handleEndSim();
  bool handleResetTrial();
  bool handleSkipTrial();
  bool handleNextTrial();

  bool resetObstacles();
  bool resetVehicles();
  bool resetOdometry();
  bool requestNewPath();
  void calcMetrics();
  bool postEndflags();

 private:  // Configuration variables
  // vehicle data
  std::string m_vehicle_name;
  XYPoint m_start_point;
  XYPoint m_goal_point;

  // interface to vehicle
  std::string m_path_request_var;
  std::string m_path_complete_var;
  std::string m_reset_sim_var;  // not set by config param

  // interface to obstacle sims
  std::vector<std::string> m_reset_obs_vars;

  // sim parameters
  int m_desired_trials;
  double m_trial_timeout;
  std::map<std::string, std::string> m_endflags;

  // default variable names
  std::string m_reset_obs_default;
  std::string m_path_complete_default;

 private:  // State variables
  bool m_sim_active;

  // sim requests
  bool m_reset_sim_pending;
  bool m_end_sim_pending;
  bool m_reset_trial_pending;
  bool m_skip_trial_pending;
  bool m_next_trial_pending;

  // trial data
  TrialData m_current_trial;
  std::vector<TrialData> m_trial_data;
};

#endif
