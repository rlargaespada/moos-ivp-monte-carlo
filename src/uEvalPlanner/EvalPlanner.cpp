/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.cpp                                 */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#include <cmath>
#include <fstream>
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
  m_end_trial_on_collision = false;
  m_export_file_base = GetAppName() + "_Metrics";
  m_use_timestamp = true;

  // state variables
  m_vspeed = 0;
  m_sim_active = false;
  m_obstacle_reset_responses = 0;
  initialize();
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
      std::string vname{tolower(msg.GetCommunity())};
      if ((isTrialOngoing())  && (vname == m_vehicle_name)) {
        if (msg.GetString() == "true") {
          m_request_new_path = SimRequest::CLOSED;
          m_next_trial_pending = true;
        }
      }
    } else if (key == m_path_failed_var) {
      std::string vname{tolower(msg.GetCommunity())};
      if ((isTrialOngoing())  && (vname == m_vehicle_name)) {
        if (msg.GetString() == "true") {
          m_request_new_path = SimRequest::CLOSED;
          m_current_trial.trial_successful = false;
          m_current_trial.planning_failed = true;
          m_next_trial_pending = true;
        }
      }
    } else if (key == "UEP_START_POS") {
      handleVPointMail(&m_start_point, msg.GetString());
    } else if (key == "UEP_GOAL_POS") {
      handleVPointMail(&m_goal_point, msg.GetString());
    // handle responses to sim requests
    } else if (key == "KNOWN_OBSTACLE_CLEAR") {
      if ((!m_reset_obs_vars.empty()) && (m_reset_obstacles == SimRequest::OPEN)) {
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

      // save current vehicle speed
      double spd{tokDoubleParse(report, "spd", ',', '=')};
      if (isTrialOngoing())
        m_current_trial.total_speed_change += std::abs(spd - m_vspeed);
      m_vspeed = spd;


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
          if (m_end_trial_on_collision) {
            m_request_new_path = SimRequest::CLOSED;
            m_next_trial_pending = true;
          }

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


void EvalPlanner::handleVPointMail(XYPoint* point, std::string point_spec)
{
  // only change start/goal points when sim is inactive
  if (m_sim_active) {
    std::string warning{"Received request to change vehicle start/goal "
                        "point while sim was active: "};
    warning += point_spec + ". ";
    warning += "Can only change start/goal point when sim is inactive.";
    reportRunWarning(warning);
    return;
  }

  // parse vname from spec; if vname doesn't match, ignore mail
  point_spec = tolower(point_spec);
  std::string vname{tokStringParse(point_spec, "vname", ',', '=')};
  if (vname != m_vehicle_name)
    return;

  // set point and post markers
  setVPoint(point, point_spec);
  postVpointMarkers();  // if point is invalid, this method will raise a warning
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
  // nothing to send if there's nothing to reset
  if (m_reset_obs_vars.empty()) {
    m_reset_obstacles = SimRequest::CLOSED;
    return (true);
  }

  // no preconditions for this request

  postFlags(m_reset_obs_vars);  // reuse this method
  m_reset_obstacles = SimRequest::OPEN;
  return (true);
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


//---------------------------------------------------------
std::string EvalPlanner::expandMacros(std::string sval)
{
  sval = macroExpand(sval, "IX", static_cast<unsigned int>(m_trial_data.size()));
  sval = macroExpand(sval, "START_X", m_start_point.x(), 2);
  sval = macroExpand(sval, "START_Y", m_start_point.y(), 2);
  sval = macroExpand(sval, "GOAL_X", m_goal_point.x(), 2);
  sval = macroExpand(sval, "GOAL_Y", m_goal_point.y(), 2);
  sval = macroExpand(sval, "V_X", m_vpos.x(), 2);
  sval = macroExpand(sval, "V_Y", m_vpos.y(), 2);

  return (sval);
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
    sval = expandMacros(sval);

    // if final val is a number, post as double
    if (isNumber(sval))
      Notify(moosvar, std::stod(sval));
    else
      Notify(moosvar, sval);

    reportEvent("Posted " + pair.getPrintable());
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
  vspec.push_back("timed_out=" + boolToString(trial.timed_out));
  vspec.push_back("planning_failed=" + boolToString(trial.planning_failed));

  vspec.push_back("planning_time=" + doubleToStringX(trial.planning_time, 2));
  vspec.push_back("duration=" + doubleToStringX(trial.duration, 2));

  vspec.push_back("encounter_count=" + intToString(trial.encounter_count));
  vspec.push_back("near_miss_count=" + intToString(trial.near_miss_count));
  vspec.push_back("collision_count=" + intToString(trial.collision_count));
  vspec.push_back("min_dist_to_obs=" + doubleToStringX(trial.min_dist_to_obs, 2));

  vspec.push_back("dist_traveled=" + doubleToStringX(trial.dist_traveled, 2));
  vspec.push_back("initial_path_len=" + doubleToStringX(trial.initial_path_len, 2));
  vspec.push_back("path_len=" + doubleToStringX(trial.path_len, 2));
  vspec.push_back("dist_eff=" + doubleToStringX(trial.dist_eff, 2));

  vspec.push_back("total_deviation=" + doubleToStringX(trial.total_deviation, 2));
  vspec.push_back("max_deviation=" + doubleToStringX(trial.max_deviation, 2));

  vspec.push_back("energy_eff=" + doubleToStringX(trial.energy_eff, 2));
  vspec.push_back("total_speed_change=" + doubleToStringX(trial.total_speed_change, 2));
  return (stringVectorToString(vspec));
}


void EvalPlanner::calcMetrics()
{
  // declare variables to track totals and extrema
  double successes{0};
  int total_timeouts{0}, total_collisions{0}, total_plan_fails{0};
  double summed_planning_time{0}, summed_duration{0};
  int num_inf_dist_to_obs{0};
  double summed_min_dist_to_obs{0}, global_min_dist_to_obs{INFINITY};
  double total_dist_traveled{0}, total_path_len{0}, summed_dist_eff{0};
  double summed_deviation{0}, global_max_deviation{0};
  double summed_energy_eff{0};
  double summed_speed_change{0};

  // iterate through trials and pull out data we need
  for (TrialData td : m_trial_data) {
    // if trial is unsuccessful, only track reason for failure
    if (!td.trial_successful) {
      if (td.timed_out)
        total_timeouts++;
      if (td.planning_failed)
        total_plan_fails++;
      total_collisions += td.collision_count;
      continue;  // don't want to compile other stats from this trial
    }

    // for successful trials, track remainder of stats
    successes++;

    summed_planning_time += td.planning_time;
    summed_duration += td.duration;

    total_collisions += td.collision_count;

    if (!std::isinf(td.min_dist_to_obs))
      summed_min_dist_to_obs += td.min_dist_to_obs;
    else
      num_inf_dist_to_obs++;
    if (td.min_dist_to_obs < global_min_dist_to_obs)
      global_min_dist_to_obs = td.min_dist_to_obs;

    total_dist_traveled += td.dist_traveled;
    total_path_len += td.path_len;
    summed_dist_eff += td.dist_eff;

    summed_deviation += td.total_deviation;
    if (td.max_deviation > global_max_deviation)
      global_max_deviation = td.max_deviation;

    summed_energy_eff += td.energy_eff;
    summed_speed_change += td.total_speed_change;
  }

  // calculate averages as needed and save to global metrics
  size_t num_trials{m_trial_data.size()};
  m_global_metrics.successful_trials = successes;
  m_global_metrics.total_trials = num_trials;
  m_global_metrics.success_rate = (successes/num_trials);

  m_global_metrics.total_timeouts = total_timeouts;
  m_global_metrics.total_collisions = total_collisions;
  m_global_metrics.total_planning_fails = total_plan_fails;

  if (successes == 0)  // nothing to calculate without a successful trial
    return;

  m_global_metrics.avg_planning_time = (summed_planning_time/successes);
  m_global_metrics.avg_duration = (summed_duration/successes);

  m_global_metrics.avg_min_dist_to_obs =
    (summed_min_dist_to_obs/(successes - num_inf_dist_to_obs));
  m_global_metrics.min_dist_to_obs = global_min_dist_to_obs;

  m_global_metrics.avg_dist_traveled = (total_dist_traveled/successes);
  m_global_metrics.avg_path_len = (total_path_len/successes);
  m_global_metrics.avg_dist_eff = (summed_dist_eff/successes);

  m_global_metrics.avg_total_deviation = (summed_deviation/successes);
  m_global_metrics.max_deviation = global_max_deviation;

  m_global_metrics.avg_energy_eff = (summed_energy_eff/successes);
  m_global_metrics.avg_total_speed_change = (summed_speed_change/successes);
}


bool EvalPlanner::exportMetrics()
{
  // return early if exports disabled or there's no trial data
  if (m_export_file_base.empty()) return (true);
  if (m_trial_data.size() == 0) return (true);

  // open export file, return if failed
  std::ofstream outf{m_export_file};
  if (!outf) {
    reportRunWarning("Unable to open " + m_export_file + " to export metrics!");
    return (false);
  }

  // add global metrics as comments in csv
  outf << "# vehicle_name=" << m_vehicle_name << "\n";
  outf << "# start_point=" << m_start_point.get_spec() << "\n";
  outf << "# goal_point=" << m_goal_point.get_spec() << "\n";
  outf << "# timeout=" << doubleToStringX(m_trial_timeout, 3)  << "\n";

  outf << "# success_rate=" << doubleToStringX(m_global_metrics.success_rate, 3) << "\n";
  outf << "# avg_planning_time=" << doubleToStringX(m_global_metrics.avg_planning_time, 3) << "\n";
  outf << "# avg_duration=" << doubleToStringX(m_global_metrics.avg_duration, 3) << "\n";
  outf << "# total_timeouts=" << intToString(m_global_metrics.total_timeouts) << "\n";
  outf << "# total_collisions=" << intToString(m_global_metrics.total_collisions) << "\n";
  outf << "# total_planning_fails=" << intToString(m_global_metrics.total_planning_fails) << "\n";

  outf << "# avg_min_dist_to_obs=" <<
    doubleToStringX(m_global_metrics.avg_min_dist_to_obs, 3) << "\n";
  outf << "# min_dist_to_obs=" << doubleToStringX(m_global_metrics.min_dist_to_obs, 3) << "\n";

  outf << "# avg_dist_traveled=" <<
    doubleToStringX(m_global_metrics.avg_dist_traveled, 3) << "\n";
  outf << "# avg_path_len=" << doubleToStringX(m_global_metrics.avg_path_len, 3) << "\n";
  outf << "# avg_dist_eff=" << doubleToStringX(m_global_metrics.avg_dist_eff, 3) << "\n";

  outf << "# deviation_limit=" << doubleToStringX(m_deviation_limit, 3) << "\n";
  outf << "# avg_total_deviation=" <<
    doubleToStringX(m_global_metrics.avg_total_deviation, 3) << "\n";
  outf << "# max_deviation=" << doubleToStringX(m_global_metrics.max_deviation, 3) << "\n";

  outf << "# avg_energy_eff=" << doubleToStringX(m_global_metrics.avg_energy_eff, 3) << "\n";

  // add trial data as csv items
  outf << "trial_num,trial_successful,timed_out, planning_failed,planning_time," <<
    "duration,collision_count,min_dist_to_obs,dist_traveled,path_len,dist_eff," <<
    "total_deviation,max_deviation,energy_eff,total_speed_change\n";

  std::vector<std::string> trialvec;
  for (TrialData td : m_trial_data) {
    trialvec.push_back(intToString(td.trial_num));
    trialvec.push_back(td.trial_successful ? "1" : "0");  // convert bool to int
    trialvec.push_back(td.timed_out ? "1" : "0");  // convert bool to int
    trialvec.push_back(td.planning_failed ? "1" : "0");  // convert bool to int

    trialvec.push_back(doubleToStringX(td.planning_time));
    trialvec.push_back(doubleToStringX(td.duration));

    trialvec.push_back(intToString(td.collision_count));
    trialvec.push_back(doubleToStringX(td.min_dist_to_obs));

    trialvec.push_back(doubleToStringX(td.dist_traveled));
    trialvec.push_back(doubleToStringX(td.path_len));
    trialvec.push_back(doubleToStringX(td.dist_eff));

    trialvec.push_back(doubleToStringX(td.total_deviation));
    trialvec.push_back(doubleToStringX(td.max_deviation));

    trialvec.push_back(doubleToStringX(td.energy_eff));
    trialvec.push_back(doubleToStringX(td.total_speed_change));

    outf << stringVectorToString(trialvec, ',') << "\n";
    trialvec.clear();
  }
  Notify("METRICS_EXPORTED", m_export_file);
  reportEvent("Exported " + GetAppName() + " metrics to: " + m_export_file);

  return (true);
}


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
    if ((m_trial_timeout > 0) && (elapsed_time > m_trial_timeout)) {
      reportEvent("Trial " + intToString(m_current_trial.trial_num) +
                  " has timed out after " + doubleToStringX(elapsed_time, 2) +
                  " seconds! Marking trial as failed.");
      m_current_trial.trial_successful = false;
      m_current_trial.timed_out = true;
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
  postFlags(m_init_flags);
  postFlags(m_trial_flags);

  // set file name for metrics export when all trials are done
  if (!m_export_file_base.empty()) {
    m_export_file = m_export_file_base;
    if (m_use_timestamp) m_export_file += "_" + MOOSGetTimeStampString();
    m_export_file += ".csv";
  }

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

  bool no_obstacle_resets{false};

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p != sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled{false};
    if (param == "vehicle_name") {
      handled = setNonWhiteVarOnString(m_vehicle_name, tolower(value));
    } else if (param == "start_pos") {
      handled = setVPoint(&m_start_point, value);
    } else if (param == "goal_pos") {
      handled = setVPoint(&m_goal_point, value);
    } else if (param == "heading_on_reset") {
      if (value == "relative") {
        m_rel_hdg_on_reset = true;
        handled = true;
      } else {handled = setDoubleOnString(m_hdg_on_reset, value);}
    } else if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, toupper(value));
    } else if (param == "path_complete_var") {
      handled = setNonWhiteVarOnString(m_path_complete_var, toupper(value));
    } else if (param == "path_stats_var") {
      handled = setNonWhiteVarOnString(m_path_stats_var, toupper(value));
    } else if (param == "path_failed_var") {
      handled = setNonWhiteVarOnString(m_path_failed_var, toupper(value));
    } else if (param == "num_trials") {
      handled = setIntOnString(m_desired_trials, value);
    } else if (param == "timeout") {
      handled = setNonNegDoubleOnString(m_trial_timeout, value);
    } else if (param == "end_trial_on_collision") {
      handled = setBooleanOnString(m_end_trial_on_collision, value);
    } else if (param == "obs_reset_var") {
      if (tolower(value) == "none") {
        no_obstacle_resets = true;
        handled = true;
      } else if (strContains(value, '=')) {
        handled = addVarDataPairOnString(m_reset_obs_vars, value);
      } else {  // add default value of "now" if we just got a variable name
        handled = addVarDataPairOnString(m_reset_obs_vars, value + "=now");
      }
    } else if (param == "deviation_limit") {
      handled = setNonNegDoubleOnString(m_deviation_limit, value);
    } else if ((param == "init_flag") || (param == "initflag")) {
      handled = addVarDataPairOnString(m_init_flags, value);
    } else if ((param == "trial_flag") || (param == "trialflag")) {
      handled = addVarDataPairOnString(m_trial_flags, value);
    } else if ((param == "end_flag") || (param == "endflag")) {
      handled = addVarDataPairOnString(m_end_flags, value);
    } else if ((param == "exportfile") || (param == "export") || (param == "file")) {
      // if param value is false/off/no, disable exports
      bool use_export{true};
      setBooleanOnString(use_export, value);  // doesn't change arg if can't parse string
      if (!use_export) {
        m_export_file_base = "";  // exports are disabled
        handled = true;
      // otherwise set export file based on parameter
      } else {
        value = stripQuotes(value);
        if (strEnds(value, ".csv")) rbiteStringX(value, '.');  // remove suffix if included
        handled = setNonWhiteVarOnString(m_export_file_base, value);
      }
    } else if ((param == "use_timestamp") || (param == "filetimestamp")) {
      handled = setBooleanOnString(m_use_timestamp, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // need a vehicle name
  if (m_vehicle_name.empty()) reportConfigWarning("Vehicle name has not been set!");

  // leave these vars unset in constructor because we don't
  // want to unnecessarily register for a variable that we don't
  // actually need
  if (m_path_complete_var.empty()) m_path_complete_var = "PATH_COMPLETE";
  if (m_path_stats_var.empty()) m_path_stats_var = "PATH_STATS";
  if (m_path_failed_var.empty())  m_path_failed_var = "PATH_FAILED";

  // if no reset vars were provided in config, only write to default var
  if ((!no_obstacle_resets) && (m_reset_obs_vars.empty()))
    m_reset_obs_vars.push_back(VarDataPair("UFOS_RESET", "now"));
  if (no_obstacle_resets) m_reset_obs_vars.clear();  // clear for safety

  // make sure start and goal were set in config file
  if (!m_start_point.valid()) reportConfigWarning("Planner start point has not been set!");
  if (!m_goal_point.valid()) reportConfigWarning("Planner goal point has not been set!");

  postVpointMarkers();
  registerVariables();
  return(true);
}


bool EvalPlanner::setVPoint(XYPoint* point, std::string point_spec)
{
  point_spec = tolower(point_spec);
  *point = string2Point(point_spec);
  return (point->valid());
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
  if (!m_path_complete_var.empty()) Register(m_path_complete_var, 0);
  if (!m_path_stats_var.empty()) Register(m_path_stats_var, 0);
  if (!m_path_failed_var.empty()) Register(m_path_failed_var, 0);

  // start/goal updates
  Register("UEP_START_POS");
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

  if (m_export_file_base.empty())
    m_msgs << "METRICS EXPORTS DISABLED" << endl;
  else if (m_sim_active)
    m_msgs << "Export CSV: " << m_export_file << endl;
  else if (m_use_timestamp)  // sim inactive but filename includes timestamp
    m_msgs << "Export CSV: " << m_export_file_base << "_<TIMESTAMP>.csv" << endl;
  else
    m_msgs << "Export CSV: " << m_export_file_base << ".csv" << endl;

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
  m_msgs << "  path_failed_var: " << m_path_failed_var << endl;
  m_msgs << "  reset_vehicle_var: " << m_reset_sim_var + "_" + upvname << endl;

  // interface to shoreside apps
  m_msgs << "Config (Interface to Obstacle Sims)" << endl;
  if (m_reset_obs_vars.empty()) {
    m_msgs << "  reset_obs_vars: NONE" << endl;
  } else {
    m_msgs << "  reset_obs_vars: ";
    for (VarDataPair pair : m_reset_obs_vars)
      m_msgs << pair.getPrintable() << ";";
    m_msgs << endl;
  }

  // high level sim state
  m_msgs << header << endl;
  m_msgs << "Sim Active: " << boolToString(m_sim_active) << endl;
  m_msgs << "Completed Trials: " << intToString(m_trial_data.size()) <<
    "/" << intToString(m_desired_trials) << endl;
  if (m_trial_timeout == 0)
    m_msgs << "Trial Timeout: DISABLED" << endl;
  else
    m_msgs << "Trial Timeout: " << doubleToStringX(m_trial_timeout, 2) << " sec" << endl;

  // global metrics over all trials
  m_msgs << header << endl;
  m_msgs << "Global Metrics" << endl;
  m_msgs << "  Success Rate: " << intToString(m_global_metrics.successful_trials) << "/" <<
    intToString(m_global_metrics.total_trials) << " (" <<
    doubleToStringX(m_global_metrics.success_rate, 2) << ")" << endl;
  m_msgs << "  Avg. Planning Time: " <<
    doubleToStringX(m_global_metrics.avg_planning_time, 2) << " s"<< endl;
  m_msgs << "  Avg. Duration: " <<
    doubleToStringX(m_global_metrics.avg_duration, 2) << " s" << endl;
  m_msgs << "  Total Timeouts: " << intToString(m_global_metrics.total_timeouts) << endl;
  m_msgs << "  Total Collisions: " << intToString(m_global_metrics.total_collisions) << endl;
  m_msgs << "  Total Planning Fails: " <<
    intToString(m_global_metrics.total_planning_fails) << endl;
  m_msgs << "  Avg. Distance Efficiency: " <<
    doubleToStringX(m_global_metrics.avg_dist_eff, 2) << endl;
  m_msgs << "  Avg. Energy Efficiency: " <<
    doubleToStringX(m_global_metrics.avg_energy_eff, 2) << endl;

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
  m_msgs << "  Total Speed Change: " << doubleToStringX(m_current_trial.total_speed_change)
    << " m/s" << endl;

  return(true);
}
