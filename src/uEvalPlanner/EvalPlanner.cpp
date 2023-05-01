/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EvalPlanner.cpp                                 */
/*    DATE: April 19th, 2023                                */
/************************************************************/

#include <iostream>
#include <iterator>
#include <string>
#include "MBUtils.h"
#include "ACTable.h"
#include "EvalPlanner.h"


//---------------------------------------------------------
// Constructor()

EvalPlanner::EvalPlanner()
{
  // config variable defaults
  m_desired_trials = 10;
  m_trial_timeout = 300;  // seconds
  m_path_request_var = "PLAN_PATH_REQUESTED";
  m_reset_sim_var = "USM_RESET";
  m_reset_obs_default = "UFOS_RESET";
  m_path_complete_default = "PATH_COMPLETE";

  // state variables
  m_sim_active = false;
  initialize();

  // todo: think about how this would work for multiple vehicles, _$V, _ALL
  // vehicles should be saved using node reports, where to define start and goal? vehicle says so?
  // USM_RESET could be set on a per vehicle basis (from node reports), use qbridge
}


void EvalPlanner::clearPendingRequests() {
  m_reset_sim_pending = false;
  m_end_sim_pending = false;
  m_reset_trial_pending = false;
  m_skip_trial_pending = false;
  m_next_trial_pending = false;
}



void EvalPlanner::clearCurrentTrialData() {
m_current_trial = TrialData{m_current_trial.trial_num};  // reuse current trial num
}


void EvalPlanner::clearCurrentTrialData(int trial_num) {
  m_current_trial = TrialData{trial_num};
}


/// @brief Initializes state variables for EvalPlanner
void EvalPlanner::initialize() {
  clearPendingRequests();
  clearCurrentTrialData(0);
  m_trial_data.clear();
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

    if (key == "RESET_SIM_REQUESTED") {
      handleSimRequest(msg.GetString(), &m_reset_sim_pending);
    } else if (key == "END_SIM_REQUESTED") {
      handleSimRequest(msg.GetString(), &m_end_sim_pending);
    } else if (key == "RESET_TRIAL_REQUESTED") {
      handleSimRequest(msg.GetString(), &m_reset_trial_pending);
    } else if (key == "SKIP_TRIAL_REQUESTED") {
      handleSimRequest(msg.GetString(), &m_skip_trial_pending);
    } else if (key == m_path_complete_var) {
      if (msg.GetString() == "true")
        m_next_trial_pending = true;
    } else if (key == "START_POS") {
      setVPoint(&m_start_point, msg.GetString());
    } else if (key == "GOAL_POS") {
      setVPoint(&m_goal_point, msg.GetString());
    } else if (key == "ENCOUNTER_ALERT") {
        std::string alert{tolower(msg.GetString())};
        std::string vname{tokStringParse(alert, "vname", ',', '=')};
        if ((m_sim_active) && (vname == m_vehicle_name)) {
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
        if ((m_sim_active) && (vname == m_vehicle_name)) {
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
        if ((m_sim_active) && (vname == m_vehicle_name)) {
          m_current_trial.collision_count++;
          m_current_trial.trial_successful = false;  // fail on collision

          double dist;
          if (tokParse(alert, "dist", ',', '=', dist)) {
            if (dist < m_current_trial.min_dist_to_obj)
              m_current_trial.min_dist_to_obj = dist;
          }
        }
    } else if (key == "PLANNING_TIME") {
      if (tolower(msg.GetCommunity()) == m_vehicle_name)
        m_current_trial.planning_time += msg.GetDouble();
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return(true);
}


void EvalPlanner::handleSimRequest(std::string request, bool* pending_flag)
{
  request = tolower(request);
  if ((request == "all") || (request == m_vehicle_name))
    *pending_flag = true;
}


//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool EvalPlanner::OnConnectToServer()
{
  registerVariables();
  return(true);
}


//---------------------------------------------------------
bool EvalPlanner::resetObstacles() {
  // todo: wait for confirmation from each sim before continuing?
  // don't want to request a new path until all obs are finalized
  bool return_val{true};

  for (std::string var_name : m_reset_obs_vars) {
    return_val = Notify(var_name, "now") && return_val;
    reportEvent("Sent " + var_name);
  }


  return (return_val);
}


bool EvalPlanner::resetVehicles() {
  bool return_val{true};

  std::string reset_pose{m_start_point.get_spec()};
  reset_pose.append(", speed=0, heading=0, depth=0");

  std::string reset_var{m_reset_sim_var + "_" + toupper(m_vehicle_name)};
  return_val = Notify(reset_var, reset_pose) && return_val;
  // todo: multiple vehicles

  std::string event;
  event += reset_var + ": " + reset_pose;
  reportEvent(event);

  return (return_val);
}


bool EvalPlanner::resetOdometry()
{
  reportEvent("Resetting trip odometry for " + m_vehicle_name);
  return (Notify("UPC_TRIP_RESET", tolower(m_vehicle_name)));
}


bool EvalPlanner::requestNewPath() {
  bool return_val{true};

  // define variable posting and message
  std::string request_var{m_path_request_var + "_" + toupper(m_vehicle_name)};
  std::string msg{"start="};
  msg += doubleToString(m_start_point.get_vx(), 2);
  msg += ',' + doubleToString(m_start_point.get_vy(), 2);
  msg += "; goal=";
  msg += doubleToString(m_goal_point.get_vx(), 2);
  msg += ',' + doubleToString(m_goal_point.get_vy(), 2);

  // post markers to start and goal
  std::string marker{"type=diamond,color=firebrick,"};
  marker += "label=" + m_vehicle_name + "_start,";
  marker += m_start_point.get_spec();
  Notify("VIEW_MARKER", marker);

  marker = "type=diamond,color=cornflowerblue,";
  marker += "label=" + m_vehicle_name + "_goal,";
  marker += m_goal_point.get_spec();
  Notify("VIEW_MARKER", marker);

  // todo: multiple vehicles

  // report event that new trial was requested
  std::string event;
  event += request_var + ": " + msg;
  reportEvent(event);

  // post new path request and save time of request
  return_val = Notify(request_var, msg);
  m_current_trial.start_time = MOOSTime();
  return (return_val);
}


bool EvalPlanner::postEndflags() {
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

  bool return_val;

  // only 1 action per iteration, in order of priority
  // executing one action clears all pending actions
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

  // if active trial has timed out, mark it a failure and start next trial
  if (m_sim_active) {
    double elapsed_time{MOOSTime() - m_current_trial.start_time};
    if (elapsed_time > m_trial_timeout) {
      reportEvent("Trial " + intToString(m_current_trial.trial_num) +
                  " has timed out after " + doubleToString(elapsed_time, 2) +
                  " seconds! Marking trial as failed.");
      m_current_trial.trial_successful = false;
      handleNextTrial();
    }
  }

  clearPendingRequests();

  AppCastingMOOSApp::PostReport();
  return (return_val);
}


bool EvalPlanner::handleEndSim() {
  if (!m_sim_active)
    return (true);

  reportEvent("Ending sim early!");
  bool return_val{true};
  // export metrics
  return_val = resetVehicles() && return_val;
  return_val = postEndflags() && return_val;

  m_sim_active = false;
  return (return_val);
}


bool EvalPlanner::handleResetSim() {
  reportEvent("Resetting simulation!");
  initialize();

  bool return_val{true};
  if (m_sim_active)
    return_val = resetObstacles() && return_val;  // only reset if already active
  return_val = resetVehicles() && return_val;
  return_val = resetOdometry() && return_val;
  return_val = requestNewPath() && return_val;

  m_sim_active = true;
  return (return_val);
}


bool EvalPlanner::handleResetTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_trial_data.size()) + " reset");
  clearCurrentTrialData();

  bool return_val{true};
  return_val = resetVehicles() && return_val;
  return_val = resetOdometry() && return_val;
  return_val = requestNewPath() && return_val;

  return (return_val);
}


bool EvalPlanner::handleSkipTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_trial_data.size()) + " skipped");
  clearCurrentTrialData();

  bool return_val{true};
  return_val = resetObstacles() && return_val;
  return_val = resetVehicles() && return_val;
  return_val = resetOdometry() && return_val;
  return_val = requestNewPath() && return_val;

  return (return_val);
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
  if (m_trial_data.size() >= m_desired_trials) {  // we're done
    reportEvent("All Monte Carlo trials complete! Setting sim to inactive.");
    resetVehicles();
    // todo: export metrics
    postEndflags();
    m_sim_active = false;
    return (true);
  }

  // more trials left to go, get ready for the next one
  clearCurrentTrialData(static_cast<int>(m_trial_data.size()));
  bool return_val{true};
  return_val = resetObstacles() && return_val;
  return_val = resetVehicles() && return_val;
  return_val = resetOdometry() && return_val;
  return_val = requestNewPath() && return_val;

  return (return_val);
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
    } else if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, value);
    } else if (param == "path_complete_var") {
      handled = setNonWhiteVarOnString(m_path_complete_var, value);
    } else if (param == "num_trials") {
      handled = setIntOnString(m_desired_trials, value);
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
    m_path_complete_var = m_path_complete_default;

  // if no reset vars were provided in config, only write to default var
  if (m_reset_obs_vars.empty())
    m_reset_obs_vars.push_back(m_reset_obs_default);

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
  return_val = tokParse(point_spec, "x", ',', '=', xval) && return_val;
  return_val = tokParse(point_spec, "y", ',', '=', yval) && return_val;

  // if spec was invalid, exit early without changing point
  if (!return_val)
    return (return_val);

  // if vname doesn't match, ignore request
  if (tolower(vname) != m_vehicle_name)
    return (return_val);

  // spec was good, change point
  point->set_vx(std::stod(xval));
  point->set_vy(std::stod(yval));
  return (return_val);
}


bool EvalPlanner::setVPointConfig(XYPoint* point, std::string point_spec)
{
  // same as setVPoint but doesn't check sim inactive, vehicle name in spec
  // parse x and y values from spec
  bool return_val{true};
  std::string vname, xval, yval;
  point_spec = tolower(point_spec);
  return_val = tokParse(point_spec, "x", ',', '=', xval) && return_val;
  return_val = tokParse(point_spec, "y", ',', '=', yval) && return_val;

  // if spec was invalid, exit early without changing point
  if (!return_val)
    return (return_val);

  // spec was good, change point
  point->set_vx(std::stod(xval));
  point->set_vy(std::stod(yval));
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
  Register("START_POS");  // todo: do this for multiple vehicles
  Register("GOAL_POS");

  // variables used to calculate metrics
  // todo: handle these subscriptions
  // success rate
  Register("ENCOUNTER_ALERT", 0);
  Register("NEAR_MISS_ALERT", 0);
  Register("COLLISION_ALERT", 0);

  // // trajectory tracking
  // Register("UPC_ODOMETRY_REPORT", 0);
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
  std::string upvname{toupper(m_vehicle_name)};
  m_msgs << "Vehicle Name: " << upvname << endl;
  m_msgs << "Sim Active: " << boolToString(m_sim_active) << endl;
  m_msgs << "Trial Timeout Cutoff: " << doubleToString(m_trial_timeout, 2) << " sec" << endl;
  m_msgs << header << endl;
  m_msgs << "Config (Interface to Planner)" << endl;
  m_msgs << "  path_request_var: "  << m_path_request_var + "_" + upvname << endl;
  m_msgs << "  path_complete_var: "  << m_path_complete_var << endl;
  m_msgs << "Config (Start and Goal)" << endl;
  m_msgs << "  start_pos: " << m_start_point.get_spec() << endl;
  m_msgs << "  goal_pos: " << m_goal_point.get_spec() << endl;
  m_msgs << "Config (Interface to Obstacle Sims)" << endl;
  m_msgs << "  reset_obs_vars: " << stringVectorToString(m_reset_obs_vars, ':') << endl;
  m_msgs << "Config (Interface to Vehicle Sims)" << endl;
  m_msgs << "  reset_sim_var: " << m_reset_sim_var + "_" + upvname << endl;
  m_msgs << header << endl;
  m_msgs << "Metrics" << endl;
  m_msgs << "  Completed Trials: " << intToString(m_trial_data.size()) <<
    "/" << intToString(m_desired_trials) << endl;
  m_msgs << header << endl;
  m_msgs << "State (Total Stats)" << endl;
  // m_msgs << "  Total Collisions: " << intToString(m_collision_count) << endl;
  // m_msgs << "  Total Near Misses: " << intToString(m_near_miss_count) << endl;
  // m_msgs << "  Total Encounters: " << intToString(m_encounter_count) << endl;
  m_msgs << "State (Trial Stats)" << endl;
  m_msgs << "  Trial Number: " << intToString(m_current_trial.trial_num) << endl;
  m_msgs << "  Trial Successful: " << toupper(boolToString(m_current_trial.trial_successful))
    << endl;
  double elapsed_time{m_sim_active ? MOOSTime() - m_current_trial.start_time : 0};  // 0 if inactive
  m_msgs << "  Elapsed Time: " << doubleToString(elapsed_time, 2) << " sec" << endl;
  m_msgs << "  Time Spent Planning: " <<
    doubleToString(m_current_trial.planning_time, 2) << " sec" << endl;
  m_msgs << "  Trial Collisions: " << intToString(m_current_trial.collision_count) << endl;
  m_msgs << "  Trial Near Misses: " << intToString(m_current_trial.near_miss_count) << endl;
  m_msgs << "  Trial Encounters: " << intToString(m_current_trial.encounter_count) << endl;
  m_msgs << "  Closest Distance to Obstacle: "
    << doubleToString(m_current_trial.min_dist_to_obj, 3) << " m" << endl;

  return(true);
}
