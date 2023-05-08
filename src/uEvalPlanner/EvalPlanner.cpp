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
#include "MacroUtils.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "EvalPlanner.h"
#include "AngleUtils.h"
#include "VarDataPair.h"


//---------------------------------------------------------
// Constructor()

EvalPlanner::EvalPlanner()
{
  // config variable defaults
  m_desired_trials = 10;
  m_trial_timeout = 300;  // seconds
  m_path_request_var = "PLAN_PATH_REQUESTED";
  m_reset_sim_var = "USM_RESET";
  m_obstacle_reset_responses = 0;
  m_rel_hdg_on_reset = false;
  m_hdg_on_reset = 0;

  // mark start/goal as invalid to start, must be set in config file
  m_start_point.invalidate();
  m_goal_point.invalidate();

  // state variables
  m_sim_active = false;
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


/// @brief Initializes state variables for EvalPlanner
void EvalPlanner::initialize()
{
  clearPendingCommands();
  clearPendingRequests();
  clearCurrentTrialData(0);
  clearTrialHistory();
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
    } else if (key == "NODE_REPORT") {
      if (m_reset_vehicles == SimRequest::OPEN) {
        if (vehicleResetComplete(msg.GetString()))
          m_reset_vehicles = SimRequest::CLOSED;
      }
    } else if (key == "UPC_ODOMETRY_REPORT") {
      std::string odo_report{tolower(msg.GetString())};
      std::string vname{tokStringParse(odo_report, "vname", ',', '=')};
      double trip_dist{tokDoubleParse(odo_report, "trip_dist", ',', '=')};
      if (m_reset_odometry == SimRequest::OPEN && vname == m_vehicle_name) {
        if (trip_dist <= 10)  // close once odo is close to 0 (allow for small error)
          m_reset_odometry = SimRequest::CLOSED;
      }
      if (isTrialOngoing() && vname == m_vehicle_name)
        m_current_trial.dist_traveled = trip_dist;
    } else if (key == "ENCOUNTER_ALERT") {
        std::string alert{tolower(msg.GetString())};
        std::string vname{tokStringParse(alert, "vname", ',', '=')};
        if ((isTrialOngoing()) && (vname == m_vehicle_name)) {
          m_current_trial.encounter_count++;

          double dist;
          if (tokParse(alert, "dist", ',', '=', dist)) {
            if (dist < m_current_trial.min_dist_to_obj)
              m_current_trial.min_dist_to_obj = dist;
          }
        }
    } else if (key == "NEAR_MISS_ALERT") {
        std::string alert{tolower(msg.GetString())};
        std::string vname{tokStringParse(alert, "vname", ',', '=')};
        if ((isTrialOngoing()) && (vname == m_vehicle_name)) {
          m_current_trial.near_miss_count++;

          double dist;
          if (tokParse(alert, "dist", ',', '=', dist)) {
            if (dist < m_current_trial.min_dist_to_obj)
              m_current_trial.min_dist_to_obj = dist;
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
            if (dist < m_current_trial.min_dist_to_obj)
              m_current_trial.min_dist_to_obj = dist;
          }
        }
    } else if (key == "PLANNING_TIME") {
      std::string vname{tolower(msg.GetCommunity())};
      if ((isTrialOngoing()) && (vname == m_vehicle_name))
        m_current_trial.planning_time += msg.GetDouble();
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


bool EvalPlanner::vehicleResetComplete(std::string node_report)
{
  node_report = tolower(node_report);
  std::string vname{tokStringParse(node_report, "name", ',', '=')};
  if (vname != m_vehicle_name)
    return (false);

  double xval{tokDoubleParse(node_report, "x", ',', '=')};
  double yval{tokDoubleParse(node_report, "y", ',', '=')};

  // reset worked if vehicle is close to start point (allow for small error)
  if (std::abs(xval - m_start_point.get_vx()) > 5)
    return (false);
  if (std::abs(yval - m_start_point.get_vy()) > 5)
    return (false);
  return (true);
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


bool EvalPlanner::cleanupSim()
{
  bool return_val{true};
  m_reset_vehicles = SimRequest::PENDING;
  // todo: export metrics
  return_val = postEndflags() && return_val;
  m_sim_active = false;
  return (return_val);
}


bool EvalPlanner::postEndflags() {
  // todo: redo this with var data pairs, macros
  // todo: add trial flags
  bool return_val{true};
  std::map<std::string, std::string>::iterator v;
  for (v = m_endflags.begin(); v != m_endflags.end(); v++) {
    return_val = Notify(v->first, v->second) && return_val;
    reportEvent("Posted " + v->first + " = " + v->second);
  }

  return (return_val);
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

  // if active trial has timed out, mark it a failure and start next trial
  if (isTrialOngoing()) {
    double elapsed_time{MOOSTime() - m_current_trial.start_time};
    if (elapsed_time > m_trial_timeout) {
      reportEvent("Trial " + intToString(m_current_trial.trial_num) +
                  " has timed out after " + doubleToStringX(elapsed_time, 2) +
                  " seconds! Marking trial as failed.");
      m_current_trial.trial_successful = false;
      handleNextTrial();
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

  return (true);
}


bool EvalPlanner::handleNextTrial() {
  if (!m_sim_active)
    return (true);

  Notify("TRIALS_COMPLETED", m_trial_data.size() + 1);
  reportEvent("Trial " + intToString(m_trial_data.size()) + " complete!");
  m_current_trial.end_time = MOOSTime();
  // calcMetrics();
  // todo: post stats, add in event?
  m_trial_data.push_back(m_current_trial);

  // if we're done, cleanup sim
  if (m_trial_data.size() >= m_desired_trials) {
    reportEvent("All Monte Carlo trials complete! Setting sim to inactive.");
    return (cleanupSim());
  }

  // more trials left to go, get ready for the next one
  clearCurrentTrialData(static_cast<int>(m_trial_data.size()));
  m_reset_obstacles = SimRequest::PENDING;
  m_reset_vehicles = SimRequest::PENDING;
  m_reset_odometry = SimRequest::PENDING;
  m_request_new_path = SimRequest::PENDING;

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
    } else if (param == "num_trials") {
      handled = setIntOnString(m_desired_trials, value);
    // todo: handle these same was as in endflags
    } else if ((param == "obs_reset_var") || (param == "obs_reset_vars")) {
      handled = handleConfigResetVars(value);
    } else if (param == "endflag") {
      handled = handleConfigEndflag(value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  if (m_vehicle_name.empty())  // need a vehicle name
    reportConfigWarning("Vehicle name has not been set!");

  // m_path_complete_var is left unset in constructor because we don't
  // want to unnecessarily register for a variable that we don't
  // actually need
  if (m_path_complete_var.empty())
    m_path_complete_var = "PATH_COMPLETE";

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

  // parse vname, x, and y values from spec
  bool return_val{true};
  std::string vname, xval, yval;
  point_spec = tolower(point_spec);
  return_val = tokParse(point_spec, "vname", ',', '=', vname) && return_val;
  // todo: use string2Point
  return_val = tokParse(point_spec, "x", ',', '=', xval) && return_val;
  return_val = tokParse(point_spec, "y", ',', '=', yval) && return_val;

  // if spec was invalid, exit early without changing point
  if (!return_val)
    return (return_val);

  // if vname doesn't match, ignore request
  if (tolower(vname) != m_vehicle_name)
    return (return_val);

  // spec was good, change point
  point->set_vertex(std::stod(xval), std::stod(yval));  // this marks point as valid

  // post new markers and return
  return_val = postVpointMarkers() && return_val;
  return (return_val);
}


bool EvalPlanner::setVPointConfig(XYPoint* point, std::string point_spec)
{
  // same as setVPoint but doesn't check sim inactive, vehicle name in spec
  // parse x and y values from spec
  bool return_val{true};
  std::string vname, xval, yval;
  point_spec = tolower(point_spec);
  // todo: use string2Point
  return_val = tokParse(point_spec, "x", ',', '=', xval) && return_val;
  return_val = tokParse(point_spec, "y", ',', '=', yval) && return_val;

  // if spec was invalid, exit early without changing point
  if (!return_val)
    return (return_val);

  // spec was good, change point
  point->set_vertex(std::stod(xval), std::stod(yval));  // this marks point as valid
  return (return_val);
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


bool EvalPlanner::handleConfigEndflag(std::string flag) {
  bool no_dupl_found{true};

  std::string var{biteStringX(flag, '=')};

  if (m_endflags.count(var) > 0)
    no_dupl_found = false;

  m_endflags[var] = flag;

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

  // high level command variables
  Register("RESET_SIM_REQUESTED", 0);
  Register("END_SIM_REQUESTED", 0);
  Register("RESET_TRIAL_REQUESTED", 0);
  Register("SKIP_TRIAL_REQUESTED", 0);
  if (!m_path_complete_var.empty())
    Register(m_path_complete_var, 0);

  // start/goal updates
  Register("UEP_START_POS");  // todo: do this for multiple vehicles
  Register("UEP_GOAL_POS");

  // sim request responses
  Register("KNOWN_OBSTACLE_CLEAR");  // todo: this should be a config var
  Register("NODE_REPORT");

  // variables used to calculate metrics
  // todo: handle these subscriptions
  // success rate
  Register("ENCOUNTER_ALERT", 0);
  Register("NEAR_MISS_ALERT", 0);
  Register("COLLISION_ALERT", 0);

  // trajectory tracking
  Register("UPC_ODOMETRY_REPORT", 0);
  // Register("UPC_SPEED_REPORT", 0);
  // Register("WPT_EFF_SUM_ALL");

  // planning time
  Register("PLANNING_TIME", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool EvalPlanner::buildReport()
{
  using std::endl;
  std::string header = "================================";

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
  m_msgs << "  path_request_var: "  << m_path_request_var + "_" + upvname << endl;
  m_msgs << "  path_complete_var: "  << m_path_complete_var << endl;
  m_msgs << "  reset_vehicle_var: " << m_reset_sim_var + "_" + upvname << endl;

  // interface to shoreside apps
  m_msgs << "Config (Interface to Obstacle Sims)" << endl;
  m_msgs << "  reset_obs_vars: " << stringVectorToString(m_reset_obs_vars, ',') << endl;

  // high level sim state
  m_msgs << header << endl;
  m_msgs << "Sim Active: " << boolToString(m_sim_active) << endl;
  m_msgs << "Completed Trials: " << intToString(m_trial_data.size()) <<
    "/" << intToString(m_desired_trials) << endl;
  m_msgs << "Trial Timeout Cutoff: " << doubleToStringX(m_trial_timeout, 2) << " sec" << endl;

  // global metrics over all trials
  m_msgs << header << endl;
  m_msgs << "Global Metrics" << endl;
  // todo: add counters for total collisions, near misses, encounters
  // m_msgs << "  Total Collisions: " << intToString(m_collision_count) << endl;
  // m_msgs << "  Total Near Misses: " << intToString(m_near_miss_count) << endl;
  // m_msgs << "  Total Encounters: " << intToString(m_encounter_count) << endl;

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
    << doubleToStringX(m_current_trial.min_dist_to_obj, 3) << " m" << endl;
  m_msgs << "  Distance Traveled: " <<
    doubleToStringX(m_current_trial.dist_traveled, 2) << " m" << endl;

  return(true);
}
