/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.h                                   */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#ifndef EvalPlanner_HEADER
#define EvalPlanner_HEADER

#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"


enum class SimRequest
{
  PENDING,
  OPEN,
  CLOSED,
};


struct TrialData
{
  int trial_num{0};
  bool trial_successful{true};

  double start_time{0};
  double end_time{0};
  double planning_time{0};

  int encounter_count{0};
  int near_miss_count{0};
  int collision_count{0};
  double min_dist_to_obj{INFINITY};

  double dist_traveled{0};

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
  void clearPendingCommands();
  void clearPendingRequests();
  void clearCurrentTrialData() {m_current_trial = TrialData{m_current_trial.trial_num};}
  void clearCurrentTrialData(int trial_num) {m_current_trial = TrialData{trial_num};}
  void clearTrialHistory() {m_trial_data.clear();}
  void initialize();

  // config handling
  bool handleConfigResetVars(std::string var_names);
  bool handleConfigEndflag(std::string flag);
  bool setVPoint(XYPoint* point, std::string point_spec);
  bool setVPointConfig(XYPoint* point, std::string point_spec);
  bool postVpointMarkers();

  // mail handling
  void handleUserCommand(std::string command, bool* pending_flag);
  bool vehicleResetComplete(std::string node_report);

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
  bool cleanupSim();
  void calcMetrics();
  std::string getMetricsSpec();
  // todo: export to a different file with each reset
  // todo: resetting sim while active should export current metrics
  bool exportMetrics();
  bool postEndflags();

 private:  // Configuration variables
  // vehicle data
  std::string m_vehicle_name;
  XYPoint m_start_point;
  XYPoint m_goal_point;
  double m_hdg_on_reset;
  bool m_rel_hdg_on_reset;

  // interface to vehicle
  std::string m_path_request_var;
  std::string m_path_complete_var;
  std::string m_reset_sim_var;  // not set by config param

  // interface to obstacle sims
  std::vector<std::string> m_reset_obs_vars;

  // sim parameters
  int m_desired_trials;
  double m_trial_timeout;  // seconds
  std::map<std::string, std::string> m_endflags;

 private:  // State variables
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
};

#endif
