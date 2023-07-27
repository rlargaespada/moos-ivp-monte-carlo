/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.h                                   */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#ifndef EvalPlanner_HEADER
#define EvalPlanner_HEADER

#include <cmath>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "VarDataPair.h"
#include "XYPoint.h"


struct TrialData
{
  int trial_num{0};  // trial index out of all trials completed
  bool trial_successful{true};  // collision, timeout, or planning failed indicates failure
  bool timed_out{false};  // trial timed out
  bool planning_failed{false};  // path planning failed

  double start_time{0};  // start time of trial
  double end_time{0};  // end time of trial
  double planning_time{0};  // total time spent planning, sourced from planner app
  double duration{0};  // trial total duration

  int encounter_count{0};  // number of encounters with obstacles
  int near_miss_count{0};  // numbrer of near misses with obstacles
  int collision_count{0};  // number of collisions with obstacles
  double min_dist_to_obs{INFINITY};  // closest aprpach to any obstacle during a trial

  double dist_traveled{0};  // total odometry during a trial
  double initial_path_len{0};  // length of first path received from planner during a trial
  double path_len{0};  // length of final path received from planner
  double dist_eff{0};  // dist_traveled / path_len

  double total_deviation{0};  // total deviation from path given by planner
  double max_deviation{0};  // maximum deviation from path given by planner

  double energy_eff{0};  // path_len / initial_path_len
  double total_speed_change{0};

  // in C++11, using default member initializers prevents brace initialization
  // so add constructors to spawn these structs
  TrialData() {}  // empty constructor uses all defaults
  explicit TrialData(int num) : trial_num{num} {};  // specify trial number
};


struct GlobalMetrics
{
  int successful_trials{0};
  int total_trials{0};
  double success_rate{1};

  int total_timeouts{0};
  int total_planning_fails{0};
  int total_collisions{0};

  double avg_planning_time{0};
  double avg_duration{0};

  double avg_min_dist_to_obs{INFINITY};
  double min_dist_to_obs{INFINITY};

  double avg_dist_traveled{0};
  double avg_path_len{0};
  double avg_dist_eff{0};

  double avg_total_deviation{0};
  double max_deviation{0};

  double avg_energy_eff{0};
  double avg_total_speed_change{0};

  // in C++11, using default member initializers prevents brace initialization
  // so add constructors to spawn these structs
  GlobalMetrics() {}  // empty constructor uses all defaults
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
  void clearPendingCommands();
  void clearPendingRequests();
  void clearCurrentTrialData(int trial_num);
  void clearCurrentTrialData() {clearCurrentTrialData(m_current_trial.trial_num);}
  void clearGlobalMetrics();
  void clearTrialHistory() {m_trial_data.clear();}
  void initialize();

  // config handling
  bool handleConfigResetVars(std::string var_names);
  bool setVPoint(XYPoint* point, std::string point_spec);
  bool postVpointMarkers();

  // mail handling
  void handleUserCommand(std::string command, bool* pending_flag);
  void handleVPointMail(XYPoint* point, std::string point_spec);
  void handlePathStats(std::string stats);

  // command handling
  bool handleResetSim();
  bool handleEndSim();
  bool handleResetTrial();
  bool handleSkipTrial();
  bool handleNextTrial();

  // request sending
  bool resetObstacles();
  bool resetVehicles();
  bool resetOdometry();
  bool requestNewPath();

  // internal actions
  bool isTrialOngoing() {return (m_request_new_path == SimRequest::OPEN);}
  std::string getTrialSpec(TrialData trial);  // turn a single trial into a string
  void calcMetrics();  // calculate overall success rate, collisions, etc.
  bool exportMetrics();  // export spec for each trial
  std::string expandMacros(std::string sval);
  void postFlags(const std::vector<VarDataPair>& flags);
  bool cleanupSim();

 private:  // request state enum
  enum class SimRequest
  {
    PENDING,
    OPEN,
    CLOSED,
  };

 private:  // Configuration variables
  // vehicle data
  std::string m_vehicle_name;
  XYPoint m_start_point;
  XYPoint m_goal_point;

  double m_hdg_on_reset;
  bool m_rel_hdg_on_reset;

  XYPoint m_prev_wpt;
  XYPoint m_next_wpt;
  double m_deviation_limit;

  // interface to vehicle
  std::string m_path_request_var;
  std::string m_path_complete_var;
  std::string m_path_stats_var;
  std::string m_path_failed_var;
  std::string m_reset_sim_var;  // not set by config param

  // interface to obstacle sims
  std::vector<VarDataPair> m_reset_obs_vars;

  // sim parameters
  int m_desired_trials;
  double m_trial_timeout;  // seconds
  bool m_end_trial_on_collision;

  std::vector<VarDataPair> m_init_flags;
  std::vector<VarDataPair> m_trial_flags;
  std::vector<VarDataPair> m_end_flags;

  // metrics export
  std::string m_export_file_base;
  bool m_use_timestamp;

 private:  // State variables
  XYPoint m_vpos;
  double m_vspeed;
  bool m_sim_active;

  // commands from user
  bool m_reset_sim_pending;
  bool m_end_sim_pending;
  bool m_reset_trial_pending;
  bool m_skip_trial_pending;

  // sim requests to other moos apps
  SimRequest m_reset_obstacles;
  int m_obstacle_reset_responses;
  SimRequest m_reset_vehicles;
  SimRequest m_reset_odometry;
  SimRequest m_request_new_path;

  // updates from vehicle
  bool m_next_trial_pending;

  // trial data
  TrialData m_current_trial;
  std::vector<TrialData> m_trial_data;
  GlobalMetrics m_global_metrics;
  std::string m_export_file;
};

#endif
