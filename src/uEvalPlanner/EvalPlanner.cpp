/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.cpp                                 */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#include <cmath>
#include <iostream>
#include <iterator>
#include <string>
#include "ACTable.h"
#include "AngleUtils.h"
#include "EvalPlanner.h"
#include "GeomUtils.h"
#include "MacroUtils.h"
#include "MBUtils.h"
#include "VarDataPair.h"
#include "VarDataPairUtils.h"
#include "XYFormatUtilsPoint.h"


//---------------------------------------------------------
// Constructor()

EvalPlanner::EvalPlanner()
{
  // config variable defaults
  m_hdg_on_reset = 0;
  m_rel_hdg_on_reset = false;
  m_deviation_limit = 5;  // meters
  m_path_request_var = "PLAN_PATH_REQUESTED";
  m_reset_sim_var = "USM_RESET";
  m_desired_trials = 10;
  m_trial_timeout = 300;  // seconds

  // state variables
  m_sim_active = false;
  m_obstacle_reset_responses = 0;
  initialize();

  // todo: think about how this would work for multiple vehicles, _$V, _ALL
  // vehicles should be saved using node reports, where to define start and goal? vehicle says so?
  // USM_RESET could be set on a per vehicle basis (from node reports), use qbridge
}


void EvalPlanner::clearPendingCommands()
{
  m_reset_sim_pending = false;
  m_end_sim_pending = false;
  m_reset_trial_pending = false;
  m_skip_trial_pending = false;
  m_next_trial_pending = false;
}


void EvalPlanner::clearPendingRequests()
{
  m_reset_obstacles = SimRequest::CLOSED;
  m_reset_vehicles = SimRequest::CLOSED;
  m_reset_odometry = SimRequest::CLOSED;
  m_request_new_path = SimRequest::CLOSED;
}


void EvalPlanner::clearCurrentTrialData(int trial_num)
{
  m_current_trial = TrialData{trial_num};
  m_prev_wpt.invalidate();
  m_next_wpt.invalidate();
}


void EvalPlanner::clearGlobalMetrics()
{
  m_trial_data.clear();
  m_global_metrics = GlobalMetrics{};
}


/// @brief Initializes state variables for EvalPlanner
void EvalPlanner::initialize()
{
  clearPendingCommands();
  clearPendingRequests();
  clearCurrentTrialData(0);
  clearTrialHistory();
  clearGlobalMetrics();
}

//---------------------------------------------------------
// Destructor

EvalPlanner::~EvalPlanner()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool EvalPlanner::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p=NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key    = msg.GetKey();

#if 0  // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    // handle user commands
    if (key == "RESET_SIM_REQUESTED") {
      handleUserCommand(msg.GetString(), &m_reset_sim_pending);
    } else if (key == "END_SIM_REQUESTED") {
      handleUserCommand(msg.GetString(), &m_end_sim_pending);
    } else if (key == "RESET_TRIAL_REQUESTED") {
      handleUserCommand(msg.GetString(), &m_reset_trial_pending);
    } else if (key == "SKIP_TRIAL_REQUESTED") {
      handleUserCommand(msg.GetString(), &m_skip_trial_pending);
    } else if (key == m_path_complete_var) {
      // todo: handle path failed
      std::string vname{tolower(msg.GetCommunity())};
      if ((isTrialOngoing())  && (vname == m_vehicle_name)) {
        if (msg.GetString() == "true") {
          m_request_new_path = SimRequest::CLOSED;
          m_next_trial_pending = true;
        }
      }
    } else if (key == "UEP_START_POS") {
      setVPoint(&m_start_point, msg.GetString());
    } else if (key == "UEP_GOAL_POS") {
      setVPoint(&m_goal_point, msg.GetString());
    // handle responses to sim requests
    } else if (key == "KNOWN_OBSTACLE_CLEAR") {
      if (m_reset_obstacles == SimRequest::OPEN) {
        m_obstacle_reset_responses += 1;
        // if we got a response from all our obstacle sims, close reset obs request
        if (m_obstacle_reset_responses == m_reset_obs_vars.size()) {
          m_reset_obstacles = SimRequest::CLOSED;
          m_obstacle_reset_responses = 0;
        }
      }
    } else if ((key == "NODE_REPORT_LOCAL") || (key == "NODE_REPORT")) {
      // check report is for correct vehicle
      std::string report{tolower(msg.GetString())};
      std::string vname{tokStringParse(report, "name", ',', '=')};
      if (vname != m_vehicle_name)
        continue;

      // save vehicle position from report
      double xval{tokDoubleParse(report, "x", ',', '=')};
      double yval{tokDoubleParse(report, "y", ',', '=')};
      m_vpos.set_vertex(xval, yval);

      // close reset request if open and vehicle is close to start
      if (m_reset_vehicles == SimRequest::OPEN) {
        if ((std::abs(xval - m_start_point.x()) < 5) &&
            (std::abs(yval - m_start_point.y()) < 5)) {
              m_reset_vehicles = SimRequest::CLOSED;
            }
      }
    } else if (key == "UPC_ODOMETRY_REPORT") {
      std::string odo_report{tolower(msg.GetString())};
      std::string vname{tokStringParse(odo_report, "vname", ',', '=')};
      if (vname != m_vehicle_name)
        continue;
      double trip_dist{tokDoubleParse(odo_report, "trip_dist", ',', '=')};
      if (m_reset_odometry == SimRequest::OPEN) {
        if (trip_dist <= 10)  // close once odo is close to 0 (allow for small error)
          m_reset_odometry = SimRequest::CLOSED;
      }
      if (isTrialOngoing())
        m_current_trial.dist_traveled = trip_dist;
    } else if (key == "ENCOUNTER_ALERT") {
        std::string alert{tolower(msg.GetString())};
        std::string vname{tokStringParse(alert, "vname", ',', '=')};
        if ((isTrialOngoing()) && (vname == m_vehicle_name)) {
          m_current_trial.encounter_count++;

          double dist;
          if (tokParse(alert, "dist", ',', '=', dist)) {
            if (dist < m_current_trial.min_dist_to_obs)
              m_current_trial.min_dist_to_obs = dist;
          }
        }
    } else if (key == "NEAR_MISS_ALERT") {
        std::string alert{tolower(msg.GetString())};
        std::string vname{tokStringParse(alert, "vname", ',', '=')};
        if ((isTrialOngoing()) && (vname == m_vehicle_name)) {
          m_current_trial.near_miss_count++;

          double dist;
          if (tokParse(alert, "dist", ',', '=', dist)) {
            if (dist < m_current_trial.min_dist_to_obs)
              m_current_trial.min_dist_to_obs = dist;
          }
        }
    } else if (key == "COLLISION_ALERT") {
        std::string alert{tolower(msg.GetString())};
        std::string vname{tokStringParse(alert, "vname", ',', '=')};
        if ((isTrialOngoing()) && (vname == m_vehicle_name)) {
          m_current_trial.collision_count++;
          m_current_trial.trial_successful = false;  // fail on collision

          double dist;
          if (tokParse(alert, "dist", ',', '=', dist)) {
            if (dist < m_current_trial.min_dist_to_obs)
              m_current_trial.min_dist_to_obs = dist;
          }
        }
    } else if (key == m_path_stats_var) {
      std::string vname{tolower(msg.GetCommunity())};
      if ((isTrialOngoing()) && (vname == m_vehicle_name))
        handlePathStats(msg.GetString());
    } else if (key == "WPT_ADVANCED") {
      // make sure it's from the right vehicle
      std::string vname{tolower(msg.GetCommunity())};
      // save previous and next waypoint
      if ((isTrialOngoing()) && (vname == m_vehicle_name)) {
        std::string prev{tokStringParse(msg.GetString(), "prev", ';', '=')};
        std::string next{tokStringParse(msg.GetString(), "next", ';', '=')};
        m_prev_wpt = string2Point(prev);
        m_next_wpt = string2Point(next);

        // if flag is posted before first waypoint is hit or after
        // last waypoint is hit, wpt behavior will give same point
        // for prev wpt and next wpt; ignore mail in these cases
        if (distPointToPoint(m_prev_wpt, m_next_wpt) == 0) {
          m_prev_wpt.invalidate();
          m_next_wpt.invalidate();
        }
      }
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return(true);
}


void EvalPlanner::handleUserCommand(std::string command, bool* pending_flag)
{
  command = tolower(command);
  if ((command == "all") || (command == m_vehicle_name))
    *pending_flag = true;
}


void EvalPlanner::handlePathStats(std::string stats)
{
  m_current_trial.planning_time += tokDoubleParse(stats, "planning_time", ',', '=');
  double path_len_traversed{tokDoubleParse(stats, "path_len_traversed", ',', '=')};
  double path_len_to_go{tokDoubleParse(stats, "path_len_to_go", ',', '=')};
  m_current_trial.path_len = (path_len_traversed + path_len_to_go);

  // first time through, set the length of the first path we got from planner
  if (m_current_trial.initial_path_len == 0)
    m_current_trial.initial_path_len = m_current_trial.path_len;
}


//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool EvalPlanner::OnConnectToServer()
{
  registerVariables();
  return(true);
}


//---------------------------------------------------------

bool EvalPlanner::resetObstacles()
{
  // no preconditions for this request
  bool return_val{true};

  for (std::string var_name : m_reset_obs_vars) {
    return_val = Notify(var_name, "now") && return_val;
    reportEvent("Sent " + var_name);
  }

  m_reset_obstacles = SimRequest::OPEN;
  return (return_val);
}


bool EvalPlanner::resetVehicles()
{
  // no preconditions for this request
  bool return_val{true};

  std::string reset_pose{m_start_point.get_spec()};
  double heading{m_rel_hdg_on_reset ? relAng(m_start_point, m_goal_point) : m_hdg_on_reset};
  reset_pose += ", speed=0, heading=" + doubleToString(heading, 2);
  reset_pose += ", depth=0";

  std::string reset_var{m_reset_sim_var + "_" + toupper(m_vehicle_name)};
  return_val = Notify(reset_var, reset_pose) && return_val;
  // todo: multiple vehicles

  std::string event;
  event += reset_var + ": " + reset_pose;
  reportEvent(event);

  m_reset_vehicles = SimRequest::OPEN;
  return (return_val);
}


bool EvalPlanner::resetOdometry()
{
  // only reset odometry after vehicles have been reset to get accurate measurements
  if (m_reset_vehicles != SimRequest::CLOSED)
    return (false);

  reportEvent("Resetting trip odometry for " + m_vehicle_name);
  Notify("UPC_TRIP_RESET", tolower(m_vehicle_name));
  m_reset_odometry = SimRequest::OPEN;
  return (true);
}


bool EvalPlanner::requestNewPath()
{
  // all other requests must be handled before requesting a new path
  if ((m_reset_obstacles != SimRequest::CLOSED) ||
      (m_reset_vehicles != SimRequest::CLOSED) ||
      (m_reset_odometry != SimRequest::CLOSED)
    ) {return (false);}

  bool return_val{true};

  // define variable posting and message
  std::string request_var{m_path_request_var + "_" + toupper(m_vehicle_name)};
  std::string msg{"start="};
  msg += doubleToStringX(m_start_point.get_vx(), 2);
  msg += ',' + doubleToStringX(m_start_point.get_vy(), 2);
  msg += "; goal=";
  msg += doubleToStringX(m_goal_point.get_vx(), 2);
  msg += ',' + doubleToStringX(m_goal_point.get_vy(), 2);

  // todo: multiple vehicles

  // report event that new trial was requested
  std::string event;
  event += request_var + ": " + msg;
  reportEvent(event);

  // post new path request and save time of request
  return_val = Notify(request_var, msg) && return_val;
  m_current_trial.start_time = MOOSTime();
  m_request_new_path = SimRequest::OPEN;
  return (return_val);
}


void EvalPlanner::postFlags(const std::vector<VarDataPair>& flags)
{
  for (VarDataPair pair : flags) {
    std::string moosvar{pair.get_var()};

    // If posting is a double, just post. No macro expansion
    if (!pair.is_string()) {
      double dval = pair.get_ddata();
      Notify(moosvar, dval);
      continue;
    }

    // Otherwise if string posting, handle macro expansion
    std::string sval{pair.get_sdata()};
    sval = macroExpand(sval, "START_X", m_start_point.x(), 2);
    sval = macroExpand(sval, "START_Y", m_start_point.y(), 2);
    sval = macroExpand(sval, "GOAL_X", m_goal_point.x(), 2);
    sval = macroExpand(sval, "GOAL_Y", m_goal_point.y(), 2);
    sval = macroExpand(sval, "V_X", m_vpos.x(), 2);
    sval = macroExpand(sval, "V_Y", m_vpos.y(), 2);

    // if final val is a number, post as double
    if (isNumber(sval))
      Notify(moosvar, std::stod(sval));
    else
      Notify(moosvar, sval);
  }
}


bool EvalPlanner::cleanupSim()
{
  m_reset_vehicles = SimRequest::PENDING;
  calcMetrics();
  exportMetrics();
  postFlags(m_end_flags);
  m_sim_active = false;
  return (true);
}


//---------------------------------------------------------

std::string EvalPlanner::getTrialSpec(TrialData trial)
{
  std::vector<std::string> vspec;

  vspec.push_back("trial_num=" + intToString(trial.trial_num));
  vspec.push_back("trial_successful=" + boolToString(trial.trial_successful));

  vspec.push_back("planning_time=" + doubleToStringX(trial.planning_time));
  vspec.push_back("duration=" + doubleToStringX(trial.duration));

  vspec.push_back("encounter_count=" + doubleToStringX(trial.encounter_count));
  vspec.push_back("near_miss_count=" + doubleToStringX(trial.near_miss_count));
  vspec.push_back("collision_count=" + doubleToStringX(trial.collision_count));
  vspec.push_back("min_dist_to_obs=" + doubleToStringX(trial.min_dist_to_obs));

  vspec.push_back("dist_traveled=" + doubleToStringX(trial.dist_traveled));
  vspec.push_back("initial_path_len=" + doubleToStringX(trial.initial_path_len));
  vspec.push_back("path_len=" + doubleToStringX(trial.path_len));
  vspec.push_back("dist_eff=" + doubleToStringX(trial.dist_eff));

  vspec.push_back("total_deviation=" + doubleToStringX(trial.total_deviation));
  vspec.push_back("max_deviation=" + doubleToStringX(trial.max_deviation));

  vspec.push_back("energy_eff=" + doubleToStringX(trial.energy_eff));
  return (stringVectorToString(vspec));
}


void EvalPlanner::calcMetrics()
{
  // set start and goal points if not set already
  if (!m_global_metrics.start_point.valid())
    m_global_metrics.start_point = m_start_point;
  if (!m_global_metrics.goal_point.valid())
    m_global_metrics.goal_point = m_goal_point;

  // declare variables to track totals and extrema
  int successes{0};
  double summed_planning_time{0}, summed_duration{0};
  int total_collisions{0};
  double summed_min_dist_to_obs{0}, global_min_dist_to_obs{INFINITY};
  double total_dist_traveled{0}, total_path_len{0}, summed_dist_eff{0};
  double summed_deviation{0}, global_max_deviation{0};
  double summed_energy_eff{0};

  // iterate through trials and pull out data we need
  for (TrialData td : m_trial_data) {
    if (td.trial_successful)
      successes++;

    summed_planning_time += td.planning_time;
    summed_duration += td.duration;

    total_collisions += td.collision_count;
    summed_min_dist_to_obs += td.min_dist_to_obs;
    if (td.min_dist_to_obs < global_min_dist_to_obs)
      global_min_dist_to_obs = td.min_dist_to_obs;

    total_dist_traveled += td.dist_traveled;
    total_path_len += td.path_len;
    summed_dist_eff += td.dist_eff;

    summed_deviation += td.total_deviation;
    if (td.max_deviation > global_max_deviation)
      global_max_deviation = td.max_deviation;

    summed_energy_eff += td.energy_eff;
  }

  // calculate averages as needed and save to global metrics
  int num_trials{m_trial_data.size()};
  m_global_metrics.success_rate = (successes/num_trials);
  m_global_metrics.avg_planning_time = (summed_planning_time/num_trials);
  m_global_metrics.avg_duration = (summed_duration/num_trials);

  m_global_metrics.total_collisions = total_collisions;
  m_global_metrics.avg_min_dist_to_obs = (summed_min_dist_to_obs/num_trials);
  m_global_metrics.min_dist_to_obs = global_min_dist_to_obs;

  m_global_metrics.avg_dist_travelled = (total_dist_traveled/num_trials);
  m_global_metrics.avg_path_len = (total_path_len/num_trials);
  m_global_metrics.avg_dist_eff = (summed_dist_eff/num_trials);

  m_global_metrics.avg_deviation = (summed_deviation/num_trials);
  m_global_metrics.max_deviation = global_max_deviation;

  m_global_metrics.avg_energy_eff = (summed_energy_eff/num_trials);
}


// todo
bool EvalPlanner::exportMetrics()
{return (true);}


//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool EvalPlanner::Iterate()
{
  AppCastingMOOSApp::Iterate();

  bool return_val{true};

  // only handle 1 command per iteration, in order of priority
  // executing one command clears all pending commands
  if (m_end_sim_pending) {
    return_val = handleEndSim();
  } else if (m_reset_sim_pending) {
    return_val = handleResetSim();
  } else if (m_reset_trial_pending) {
    return_val = handleResetTrial();
  } else if (m_skip_trial_pending) {
    return_val = handleSkipTrial();
  } else if (m_next_trial_pending) {
    return_val = handleNextTrial();
  }
  clearPendingCommands();

  // update some stats each iteration trial is running
  if (isTrialOngoing()) {
    // if active trial has timed out, mark it a failure and start next trial
    double elapsed_time{MOOSTime() - m_current_trial.start_time};
    if (elapsed_time > m_trial_timeout) {
      reportEvent("Trial " + intToString(m_current_trial.trial_num) +
                  " has timed out after " + doubleToStringX(elapsed_time, 2) +
                  " seconds! Marking trial as failed.");
      m_current_trial.trial_successful = false;
      m_request_new_path = SimRequest::CLOSED;
      handleNextTrial();
    }

    // calc and save perpendicular deviation from path
    // NOTE: artificially boosted by vehicle capture radius
    // causing waypoint tracking errors
    if ((m_prev_wpt.valid()) && (m_next_wpt.valid())) {
      double deviation{std::abs(distPointToSeg(
        m_prev_wpt.x(), m_prev_wpt.y(),
        m_next_wpt.x(), m_next_wpt.y(),
        m_vpos.x(), m_vpos.y()))
      };
      if (deviation > m_deviation_limit) {
        m_current_trial.total_deviation += deviation - m_deviation_limit;
        if (deviation > m_current_trial.max_deviation)
          m_current_trial.max_deviation = deviation;
      }
    }
  }

  // send requests to other apps as needed
  if (m_reset_obstacles == SimRequest::PENDING)
    resetObstacles();
  if (m_reset_vehicles == SimRequest::PENDING)
    resetVehicles();
  if (m_reset_odometry == SimRequest::PENDING)
    resetOdometry();
  if (m_request_new_path == SimRequest::PENDING)
    requestNewPath();

  AppCastingMOOSApp::PostReport();
  return (return_val);
}


bool EvalPlanner::handleEndSim() {
  if (!m_sim_active)
    return (true);

  reportEvent("Ending sim early!");
  return (cleanupSim());
}


bool EvalPlanner::handleResetSim() {
  reportEvent("Resetting simulation!");
  initialize();

  if (m_sim_active)
    m_reset_obstacles = SimRequest::PENDING;  // only reset if already active
  m_reset_vehicles = SimRequest::PENDING;
  m_reset_odometry = SimRequest::PENDING;
  m_request_new_path = SimRequest::PENDING;
  postFlags(m_trial_flags);

  m_sim_active = true;
  return (true);
}


bool EvalPlanner::handleResetTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_trial_data.size()) + " reset");
  clearCurrentTrialData();

  m_reset_vehicles = SimRequest::PENDING;
  m_reset_odometry = SimRequest::PENDING;
  m_request_new_path = SimRequest::PENDING;
  postFlags(m_trial_flags);

  return (true);
}


bool EvalPlanner::handleSkipTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_trial_data.size()) + " skipped");
  clearCurrentTrialData();

  m_reset_obstacles = SimRequest::PENDING;
  m_reset_vehicles = SimRequest::PENDING;
  m_reset_odometry = SimRequest::PENDING;
  m_request_new_path = SimRequest::PENDING;
  postFlags(m_trial_flags);

  return (true);
}


bool EvalPlanner::handleNextTrial() {
  if (!m_sim_active)
    return (true);

  // post trial complete
  Notify("TRIALS_COMPLETED", m_trial_data.size() + 1);
  reportEvent("Trial " + intToString(m_trial_data.size()) + " complete!");

  // calculate and post final metrics now that trial is done
  m_current_trial.end_time = MOOSTime();
  m_current_trial.duration = (m_current_trial.end_time - m_current_trial.start_time);
  m_current_trial.dist_eff = (m_current_trial.dist_traveled/m_current_trial.path_len);
  m_current_trial.energy_eff = (m_current_trial.path_len/m_current_trial.initial_path_len);
  Notify("TRIAL_STATS", getTrialSpec(m_current_trial));
  m_trial_data.push_back(m_current_trial);

  // if we're done, cleanup sim
  if (m_trial_data.size() >= m_desired_trials) {
    reportEvent("All Monte Carlo trials complete! Setting sim to inactive.");
    return (cleanupSim());
  }

  // more trials left to go, get ready for the next one
  calcMetrics();  // calc global metrics so far to put in appcast
  clearCurrentTrialData(static_cast<int>(m_trial_data.size()));
  m_reset_obstacles = SimRequest::PENDING;
  m_reset_vehicles = SimRequest::PENDING;
  m_reset_odometry = SimRequest::PENDING;
  m_request_new_path = SimRequest::PENDING;
  postFlags(m_trial_flags);

  return (true);
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool EvalPlanner::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p != sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled{false};
    if (param == "vehicle_name") {
      handled = setNonWhiteVarOnString(m_vehicle_name, value);
    } else if (param == "timeout") {
      handled = setPosDoubleOnString(m_trial_timeout, value);
    } else if (param == "start_pos") {
      handled = setVPointConfig(&m_start_point, value);
    } else if (param == "goal_pos") {
      handled = setVPointConfig(&m_goal_point, value);
    } else if (param == "heading_on_reset") {
      if (value == "relative") {
        m_rel_hdg_on_reset = true;
        handled = true;
      } else {handled = setDoubleOnString(m_hdg_on_reset, value);}
    } else if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, value);
    } else if (param == "path_complete_var") {
      handled = setNonWhiteVarOnString(m_path_complete_var, value);
    } else if (param == "path_stats_var") {
      handled = setNonWhiteVarOnString(m_path_stats_var, value);
    } else if (param == "num_trials") {
      handled = setIntOnString(m_desired_trials, value);
    } else if ((param == "obs_reset_var") || (param == "obs_reset_vars")) {
      handled = handleConfigResetVars(value);
    } else if (param == "deviation_limit") {
      handled = setNonNegDoubleOnString(m_deviation_limit, value);
    } else if ((param == "trial_flag") || (param == "trialflag")) {
      handled = addVarDataPairOnString(m_trial_flags, value);
    } else if ((param == "end_flag") || (param == "endflag")) {
      handled = addVarDataPairOnString(m_end_flags, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  if (m_vehicle_name.empty())  // need a vehicle name
    reportConfigWarning("Vehicle name has not been set!");

  // leave these vars unset in constructor because we don't
  // want to unnecessarily register for a variable that we don't
  // actually need
  if (m_path_complete_var.empty())
    m_path_complete_var = "PATH_COMPLETE";
  if (m_path_stats_var.empty())
    m_path_stats_var = "PATH_STATS";

  // if no reset vars were provided in config, only write to default var
  if (m_reset_obs_vars.empty())
    m_reset_obs_vars.push_back("UFOS_RESET");

  // make sure start and goal were set in config file
  if (!m_start_point.valid())
    reportConfigWarning("Planner start point has not been set!");
  if (!m_goal_point.valid())
    reportConfigWarning("Planner goal point has not been set!");

  postVpointMarkers();
  registerVariables();
  return(true);
}


bool EvalPlanner::setVPoint(XYPoint* point, std::string point_spec)
{
  // only change start/goal points when sim is inactive
  if (m_sim_active) {
    std::string warning{"Received request to change vehicle start/goal "
                        "point while sim was active: "};
    warning += point_spec + ". ";
    warning += "Can only change start/goal point when sim is inactive.";
    reportRunWarning(warning);
    return (false);
  }

  // parse vname from spec; if vname doesn't match, ignore request
  point_spec = tolower(point_spec);
  std::string vname{tokStringParse(point_spec, "vname", ',', '=')};
  if (vname != m_vehicle_name)
    return (false);

  // parse point from string and return validity
  *point = string2Point(point_spec);
  if (point-> valid())
    postVpointMarkers();
  return (point->valid());
}


bool EvalPlanner::setVPointConfig(XYPoint* point, std::string point_spec)
{
  // same as setVPoint but doesn't check sim inactive, vehicle name in spec
  // parse x and y values from spec
  bool return_val{true};
  std::string vname, xval, yval;
  point_spec = tolower(point_spec);
  *point = string2Point(point_spec);
  return (point->valid());
}


bool EvalPlanner::handleConfigResetVars(std::string var_names) {
  bool no_dupl_found{true};

  var_names = stripBlankEnds(var_names);
  std::vector<std::string> svector{parseString(var_names, ',')};
  for (std::string var_name : svector) {
    var_name = stripBlankEnds(var_name);
    if (vectorContains(m_reset_obs_vars, var_name))
      no_dupl_found = false;
    else
      m_reset_obs_vars.push_back(var_name);
  }

  return (no_dupl_found);
}


bool EvalPlanner::postVpointMarkers()
{
  bool return_val{true};
  std::string marker;

  // post start marker
  if (m_start_point.valid()) {
    marker = "type=diamond,color=firebrick,";
    marker += "label=" + m_vehicle_name + "_start,";
    marker += m_start_point.get_spec();
    return_val = Notify("VIEW_MARKER", marker) && return_val;
  } else {
    reportRunWarning("Can't post marker for planner start point, start point is invalid");
    return_val = false;
  }

  // post goal marker
  if (m_start_point.valid()) {
    marker = "type=diamond,color=cornflowerblue,";
    marker += "label=" + m_vehicle_name + "_goal,";
    marker += m_goal_point.get_spec();
    return_val = Notify("VIEW_MARKER", marker) && return_val;
  } else {
    reportRunWarning("Can't post marker for planner goal point, start goal is invalid");
    return_val = false;
  }

  return (return_val);
}


//---------------------------------------------------------
// Procedure: registerVariables()

void EvalPlanner::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // user commands
  Register("RESET_SIM_REQUESTED", 0);
  Register("END_SIM_REQUESTED", 0);
  Register("RESET_TRIAL_REQUESTED", 0);
  Register("SKIP_TRIAL_REQUESTED", 0);

  // planner interface
  if (!m_path_complete_var.empty())
    Register(m_path_complete_var, 0);
  if (!m_path_stats_var.empty())
    Register(m_path_stats_var, 0);

  // start/goal updates
  Register("UEP_START_POS");  // todo: do this for multiple vehicles
  Register("UEP_GOAL_POS");

  // sim request responses
  Register("KNOWN_OBSTACLE_CLEAR");
  Register("NODE_REPORT");
  Register("NODE_REPORT_LOCAL");

  // variables used to calculate metrics
  // success rate
  Register("ENCOUNTER_ALERT", 0);
  Register("NEAR_MISS_ALERT", 0);
  Register("COLLISION_ALERT", 0);

  // trajectory tracking
  Register("UPC_ODOMETRY_REPORT", 0);
  Register("WPT_ADVANCED", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool EvalPlanner::buildReport()
{
  using std::endl;
  std::string header{"================================"};

  // high level config
  std::string upvname{toupper(m_vehicle_name)};
  m_msgs << "Vehicle Name: " << upvname << endl;
  // todo: add metrics export file
  std::string start_pose{m_start_point.get_spec()};
  double heading{m_rel_hdg_on_reset ? relAng(m_start_point, m_goal_point) : m_hdg_on_reset};
  start_pose += ", heading=" + doubleToStringX(heading, 2);
  m_msgs << "Start Pose: " << start_pose << endl;
  m_msgs << "Goal Point: " << m_goal_point.get_spec() << endl;

  // interface to vehicle apps
  m_msgs << header << endl;
  m_msgs << "Config (Interface to Vehicle)" << endl;
  m_msgs << "  path_request_var: " << m_path_request_var + "_" + upvname << endl;
  m_msgs << "  path_complete_var: " << m_path_complete_var << endl;
  m_msgs << "  path_stats_var: " << m_path_stats_var << endl;
  m_msgs << "  reset_vehicle_var: " << m_reset_sim_var + "_" + upvname << endl;

  // interface to shoreside apps
  m_msgs << "Config (Interface to Obstacle Sims)" << endl;
  m_msgs << "  reset_obs_vars: " << stringVectorToString(m_reset_obs_vars, ',') << endl;

  // high level sim state
  m_msgs << header << endl;
  m_msgs << "Sim Active: " << boolToString(m_sim_active) << endl;
  m_msgs << "Completed Trials: " << intToString(m_trial_data.size()) <<
    "/" << intToString(m_desired_trials) << endl;
  m_msgs << "Trial Time Limit: " << doubleToStringX(m_trial_timeout, 2) << " sec" << endl;

  // global metrics over all trials
  m_msgs << header << endl;
  m_msgs << "Global Metrics" << endl;
  // todo: add counters for total collisions, near misses, encounters
  // todo: maybe these could be in a table?

  // metrics for current trial
  m_msgs << header << endl;
  m_msgs << "Trial Metrics" << endl;
  m_msgs << "  Trial Number: " << intToString(m_current_trial.trial_num) << endl;
  m_msgs << "  Trial Successful: " << toupper(boolToString(m_current_trial.trial_successful))
    << endl;
  double elapsed_time{isTrialOngoing() ? MOOSTime() - m_current_trial.start_time : 0};
  m_msgs << "  Elapsed Time: " << doubleToStringX(elapsed_time, 2) << " sec" << endl;
  m_msgs << "  Time Spent Planning: " <<
    doubleToStringX(m_current_trial.planning_time, 2) << " sec" << endl;
  m_msgs << "  Trial Collisions: " << intToString(m_current_trial.collision_count) << endl;
  m_msgs << "  Trial Near Misses: " << intToString(m_current_trial.near_miss_count) << endl;
  m_msgs << "  Trial Encounters: " << intToString(m_current_trial.encounter_count) << endl;
  m_msgs << "  Closest Approach to Any Obstacle: "
    << doubleToStringX(m_current_trial.min_dist_to_obs, 2) << " m" << endl;
  m_msgs << "  Distance Traveled: " <<
    doubleToStringX(m_current_trial.dist_traveled, 2) << " m" << endl;
  m_msgs << "  Path Length: " <<
    doubleToStringX(m_current_trial.path_len, 2) << " m" << endl;
  m_msgs << "  Initial Path Length: " <<
    doubleToStringX(m_current_trial.initial_path_len, 2) << " m" << endl;
  m_msgs << "  Total Deviation > " << doubleToStringX(m_deviation_limit, 2) <<
    " m from Path: " << doubleToStringX(m_current_trial.total_deviation) << " m" << endl;
  m_msgs << "  Maximum Deviation > " << doubleToStringX(m_deviation_limit, 2) <<
    " m from Path: " << doubleToStringX(m_current_trial.max_deviation) << " m" << endl;

  return(true);
}
