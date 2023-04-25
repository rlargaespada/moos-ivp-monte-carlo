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
  m_sim_active = false;

  // todo: think about how this would work for multiple vehicles, _$V, _ALL
  // vehicles should be saved using node reports, where to define start and goal? vehicle says so?
  // USM_RESET could be set on a per vehicle basis (from node reports), use qbridge

  // set up config defaults
  m_reset_obs_default = "UFOS_RESET";
  m_path_complete_default = "PATH_COMPLETE";
  m_path_request_var = "PLAN_PATH_REQUESTED";
  m_reset_sim_var = "USM_RESET";
  m_desired_trials = 10;

  initialize();
}


void EvalPlanner::clearPendingRequests() {
  m_reset_sim_pending = false;
  m_end_sim_pending = false;

  m_reset_trial_pending = false;
  m_skip_trial_pending = false;
  m_next_trial_pending = false;
}


void EvalPlanner::clearMetrics() {}  // todo: add metrics


void EvalPlanner::clearTotalCounts() {
  m_encounter_count = 0;
  m_near_miss_count = 0;
  m_collision_count = 0;

  m_completed_trials = 0;
}


void EvalPlanner::clearTrialData() {
  // todo: also need to clear metrics
  m_encounter_count_trial = 0;
  m_near_miss_count_trial = 0;
  m_collision_count_trial = 0;
}


/// @brief Initializes state variables for EvalPlanner
void EvalPlanner::initialize() {
  clearPendingRequests();
  clearMetrics();
  clearTotalCounts();
  clearTrialData();
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
      m_reset_sim_pending = true;
    } else if (key == "END_SIM_REQUESTED") {
      m_end_sim_pending = true;
    } else if (key == "RESET_TRIAL_REQUESTED") {
      m_reset_trial_pending = true;
    } else if (key == "SKIP_TRIAL_REQUESTED") {
      m_skip_trial_pending = true;
    } else if (key == m_path_complete_var) {
      if (msg.GetString() == "true")
        m_next_trial_pending = true;
    } else if (key == "START_POS") {
      setVPoint(&m_start_point, msg.GetString());
    } else if (key == "GOAL_POS") {
      setVPoint(&m_goal_point, msg.GetString());
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
    // todo: new collisions should be immediately counted here, not in calcMetrics
  }

  return(true);
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

  return_val = Notify(m_reset_sim_var, reset_pose) && return_val;
  // todo: multiple vehicles

  std::string event;
  event += m_reset_sim_var + ": " + reset_pose;
  reportEvent(event);

  return (return_val);
}


bool EvalPlanner::requestNewPath() {
  bool return_val{true};

  std::string msg{"start="};
  msg += doubleToString(m_start_point.get_vx());
  msg += ',' + doubleToString(m_start_point.get_vy());
  msg += "; goal=";
  msg += doubleToString(m_goal_point.get_vx());
  msg += ',' + doubleToString(m_goal_point.get_vy());

  return_val = Notify(m_path_request_var, msg);

  // add markers to start and goal
  std::string marker{"type=diamond,label=start,color=firebrick,"};
  marker += m_start_point.get_spec();
  Notify("VIEW_MARKER", marker);

  marker = "type=diamond,label=goal,color=cornflowerblue,";
  marker += m_goal_point.get_spec();
  Notify("VIEW_MARKER", marker);

  // todo: multiple vehicles

  std::string event;
  event += m_path_request_var + ": " + msg;
  reportEvent(event);

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

  clearPendingRequests();

  AppCastingMOOSApp::PostReport();
  return (return_val);
}


bool EvalPlanner::handleEndSim() {
  if (!m_sim_active)
    return (true);

  reportEvent("Ending sim early!");
  bool return_val{true};
  m_sim_active = false;
  return_val = resetVehicles() && return_val;
  return_val = postEndflags() && return_val;
  return (return_val);
}


bool EvalPlanner::handleResetSim() {
  reportEvent("Resetting simulation!");
  initialize();

  bool return_val{true};
  if (m_sim_active)
    return_val = resetObstacles() && return_val;  // only reset if already active
  return_val = resetVehicles() && return_val;
  return_val = requestNewPath() && return_val;

  m_sim_active = true;
  return (return_val);
}


bool EvalPlanner::handleResetTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_completed_trials) + " reset");
  clearTrialData();

  bool return_val{true};
  return_val = resetVehicles() && return_val;
  return_val = requestNewPath() && return_val;

  return (return_val);
}


bool EvalPlanner::handleSkipTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_completed_trials) + " skipped");
  clearTrialData();

  bool return_val{true};
  return_val = resetObstacles() && return_val;
  return_val = resetVehicles() && return_val;
  return_val = requestNewPath() && return_val;

  return (return_val);
}


bool EvalPlanner::handleNextTrial() {
  if (!m_sim_active)
    return (true);

  reportEvent("Trial " + intToString(m_completed_trials) + " complete!");
  // calcMetrics();
  clearTrialData();

  m_completed_trials += 1;
  if (m_completed_trials >= m_desired_trials) {  // we're done
    reportEvent("All Monte Carlo trials complete! Setting sim to inactive.");
    resetVehicles();
    m_sim_active = false;
    postEndflags();
    return (true);
  }

  bool return_val{true};
  return_val = resetVehicles() && return_val;
  return_val = resetObstacles() && return_val;
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
    if (param == "start_pos") {
      handled = setVPoint(&m_start_point, value);
    } else if (param == "goal_pos") {
      handled = setVPoint(&m_goal_point, value);
    } else if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, value);
    } else if (param == "path_complete_var") {
      handled = setNonWhiteVarOnString(m_path_complete_var, value);
    } else if (param == "num_trials") {
      m_desired_trials = std::stoi(value);
      handled = true;
    } else if ((param == "obs_reset_var") || (param == "obs_reset_vars")) {
      handled = handleConfigResetVars(value);
    } else if (param == "endflag") {
      handled = handleConfigEndflag(value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

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
  // parse x and y values from spec
  bool return_val{true};
  std::string xval, yval;
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
  std::vector<std::string> svector{parseString(var_names, ':')};
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
  // Register("OB_ENCOUNTER", 0);
  // Register("OB_NEAR_MISS", 0);
  // Register("OB_COLLISION", 0);
  // Register("ENCOUNTER_COUNT", 0);
  // Register("NEAR_MISS_COUNT", 0);
  // Register("COLLISION_COUNT", 0);

  // // trajectory tracking
  // Register("UPC_ODOMETRY_REPORT", 0);
  // Register("WPT_EFF_SUM_ALL");

  // // plannint time
  // Register("PLANNING_TIME", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool EvalPlanner::buildReport()
{
  using std::endl;
  std::string header = "================================";
  m_msgs << "Sim Active: " << boolToString(m_sim_active) << endl;
  m_msgs << header << endl;
  m_msgs << "Config (Interface to Planner)" << endl;
  m_msgs << "  path_request_var:   "  << m_path_request_var << endl;
  m_msgs << "  path_complete_var:   "  << m_path_complete_var << endl;
  m_msgs << "Config (Start and Goal)" << endl;
  m_msgs << "  start_pos:   " << m_start_point.get_spec() << endl;
  m_msgs << "  goal_pos:   " << m_goal_point.get_spec() << endl;
  m_msgs << "Config (Interface to Obstacle Sims)" << endl;
  m_msgs << "  reset_obs_vars:   " << stringVectorToString(m_reset_obs_vars, ':') << endl;
  m_msgs << "Config (Interface to Vehicle Sims)" << endl;
  m_msgs << "  reset_sim_var:   " << m_reset_sim_var << endl;
  m_msgs << header << endl;
  m_msgs << "Metrics" << endl;
  m_msgs << "  Completed Trials:   " << intToString(m_completed_trials) <<
    "/" << intToString(m_desired_trials) << endl;
  m_msgs << header << endl;
  m_msgs << "State (Total Stats)" << endl;
  m_msgs << "  Total Collisions:   " << intToString(m_collision_count) << endl;
  m_msgs << "  Total Near Misses:   " << intToString(m_near_miss_count) << endl;
  m_msgs << "  Total Encounters:   " << intToString(m_encounter_count) << endl;
  m_msgs << "State (Trial Stats)" << endl;
  m_msgs << "  Trial Collisions:   " << intToString(m_collision_count_trial) << endl;
  m_msgs << "  Trial Near Misses:   " << intToString(m_near_miss_count_trial) << endl;
  m_msgs << "  Trial Encounters:   " << intToString(m_encounter_count_trial) << endl;

  return(true);
}
