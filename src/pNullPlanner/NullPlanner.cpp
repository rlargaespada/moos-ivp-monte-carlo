/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NullPlanner.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "NullPlanner.h"
#include "MacroUtils.h"
#include "MBUtils.h"
#include "VarDataPair.h"
#include "VarDataPairUtils.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsSegl.h"
#include "XYPoint.h"
#include "XYPolygon.h"


//---------------------------------------------------------
// Constructor()

NullPlanner::NullPlanner()
{
  //* Config Variables
  // vars to subscribe to, all are set in onStartup()
  m_path_request_var = "";
  m_obs_alert_var = "";
  m_wpt_complete_var = "";

  // publication config
  m_prefix = "";
  m_path_found_var = "PATH_FOUND";
  m_path_complete_var = "PATH_COMPLETE";
  m_path_stats_var = "PATH_STATS";
  m_path_failed_var = "PATH_FAILED";

  //* State Variables
  // planning state data
  m_mode = PlannerMode::IDLE;
  m_planning_start_time = 0;
  m_planning_end_time = 0;
  m_path_len_traversed = 0;

  // seed generator using MOOS time
  m_generator.seed(MOOSTime());
}

//---------------------------------------------------------
// Destructor

NullPlanner::~NullPlanner()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool NullPlanner::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
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

    if (key == m_path_request_var) {
      std::string request{msg.GetString()};
      reportEvent("Path Requested: " + request);
      if (setEndpoints(request))
        m_mode = PlannerMode::REQUEST_PENDING;
      else
        reportRunWarning("Invalid " + key + ": " + request);
    } else if (key == m_obs_alert_var) {
      handleObstacleAlert(msg.GetString());
    } else if (key == "OBM_RESOLVED") {
      handleObstacleResolved(msg.GetString());
    } else if (key == m_wpt_complete_var) {
      if (m_mode == PlannerMode::IN_TRANSIT) {
        m_mode = PlannerMode::PATH_COMPLETE;

        // add final segment of path to dist traveled
        unsigned int last_idx{m_path.size() - 1};
        double x1{m_path.get_vx(last_idx)}, y1{m_path.get_vy(last_idx)};
        double x0{m_path.get_vx(last_idx - 1)}, y0{m_path.get_vy(last_idx - 1)};
        m_path_len_traversed += hypot(x1 - x0, y1 - y0);
      }
    } else if ((key == "NODE_REPORT_LOCAL") || (key == "NODE_REPORT")) {
      std::string report{tolower(msg.GetString())};
      double xval{tokDoubleParse(report, "x", ',', '=')};
      double yval{tokDoubleParse(report, "y", ',', '=')};
      m_vpos.set_vertex(xval, yval);
    } else if (key == "WPT_INDEX") {
      // add just finished segment of path to dist traveled
      int prev_idx{static_cast<int>(msg.GetDouble()) - 1};
      if ((m_mode == PlannerMode::IN_TRANSIT)  && (prev_idx > 0)) {
        double x1{m_path.get_vx(prev_idx)}, y1{m_path.get_vy(prev_idx)};
        double x0{m_path.get_vx(prev_idx - 1)}, y0{m_path.get_vy(prev_idx - 1)};
        m_path_len_traversed += hypot(x1 - x0, y1 - y0);
      }
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return(true);
}


bool NullPlanner::setEndpoints(std::string request)
{
  std::string start, goal;
  request = tolower(request);

  // pull out start and goal, return fail if can't find
  start = tokStringParse(request, "start", ';', '=');
  goal = tokStringParse(request, "goal", ';', '=');
  if ((start.empty()) || (goal.empty()))
    return (false);

  // apply spec to start and goal, if parsing fails return fail
  m_start_point = string2Point(start);
  m_goal_point = string2Point(goal);
  if (!m_start_point.valid() || !m_goal_point.valid())
    return (false);

  return (true);
}


bool NullPlanner::handleObstacleAlert(std::string obs_alert)
{
  // name=avd_obstacles_ob_0#poly=pts={-21.69,-135.31:-24.25,-132.75:-10.83,-132.03},label=ob_0
  // pull out polygon from alert message
  biteStringX(obs_alert, '#');  // removes name block
  biteStringX(obs_alert, '=');  // removes poly= block, just leaves pts=

  // parse obstacle from string
  XYPolygon new_obs{string2Poly(obs_alert)};
  if (!new_obs.is_convex())
    return(false);

  // add new obstacle to the ADD queue
  std::string key{new_obs.get_label()};
  m_obstacle_add_queue[key] = new_obs;

  // if this is an obstacle with a label we've seen before,
  // add it to the REMOVE queue so we can remove the old
  // version of the obstacle in the iterate loop
  if (m_obstacle_map.count(key))
    m_obstacle_remove_queue.insert(key);

  return (true);
}


bool NullPlanner::handleObstacleResolved(const std::string obs_label)
{
  // if obs is in ADD queue for some reason, remove it
  m_obstacle_add_queue.erase(obs_label);

  // if obstacle is in obstacle map, add to REMOVE queue
  if (m_obstacle_map.count(obs_label)) {
    m_obstacle_remove_queue.insert(obs_label);
    return (true);
  }
  // otherwise ignore message
  return (false);
}


//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool NullPlanner::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool NullPlanner::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // refresh obstacle grid each iteration if we're not
  // actively planning
  if (m_mode != PlannerMode::PLANNING_IN_PROGRESS) syncObstacles();

  bool path_found{false};

  // new path request
  if (m_mode == PlannerMode::REQUEST_PENDING) {
    // post new plan messages
    Notify(m_prefix + m_path_found_var, "false");
    Notify(m_prefix + m_path_complete_var, "false");
    Notify(m_prefix + m_path_failed_var, "false");
    postFlags(m_init_plan_flags);

    // clear planning state data
    m_planning_start_time = MOOSTime();
    m_path_len_traversed = 0;
    m_path.clear();

    // plan path until we reach max number of iterations
    path_found = planPath();
  }

  // notify that path has been found
  if (path_found) {
    m_planning_end_time = MOOSTime();
    m_mode = PlannerMode::IN_TRANSIT;
    postPath();
  }

  // if path is complete, cleanup
  if (m_mode == PlannerMode::PATH_COMPLETE) {
    Notify(m_prefix + m_path_complete_var, "true");
    postFlags(m_end_flags);
    m_mode = PlannerMode::IDLE;
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}


//---------------------------------------------------------

bool NullPlanner::checkPlanningPreconditions()
{
  std::vector<std::string> warnings;

  // check all preconditions
  if (!m_start_point.valid() || !m_goal_point.valid())
    warnings.push_back("Start or goal point not valid!");

  // no warnings, we're good
  if (warnings.empty())
    return (true);

  // otherwise, post warnings
  reportRunWarning("Cannot plan, " + GetAppName() +
    " is configured incorrectly for the following reasons:");
  for (std::string warning : warnings)
    reportRunWarning(warning);
  return (false);
}


void NullPlanner::syncObstacles()
{
  for (auto const& obs : m_obstacle_remove_queue)
    m_obstacle_map.erase(obs);
  for (auto const& obs : m_obstacle_add_queue)
    m_obstacle_map[obs.first] = obs.second;
  m_obstacle_add_queue.clear();
  m_obstacle_remove_queue.clear();
}


bool NullPlanner::planPath()
{
  if (!checkPlanningPreconditions()) {
    handlePlanningFail();  // checkPlanningPreconditions posts its own warnings
    return (false);
  }

  // path is just start point, any intermediate points, and goal point
  m_path.add_vertex(m_start_point);

  // add intermediate points to path
  // include small perturbation so that downstream waypoint behavior
  // is forced to reset when path is published
  for (int i = 0; i < m_intermediate_pts.size(); i ++)
    m_path.add_vertex(m_intermediate_pts.get_vx(i) + m_perturbation(m_generator),
                      m_intermediate_pts.get_vy(i) + m_perturbation(m_generator));

  m_path.add_vertex(m_goal_point);

  return (true);
}


void NullPlanner::handlePlanningFail(std::string warning_msg)
{
  if (!warning_msg.empty()) reportRunWarning(warning_msg);
  reportRunWarning("Planning failed!");
  m_mode = PlannerMode::PLANNING_FAILED;
  Notify(m_prefix + m_path_failed_var, "true");
}


//---------------------------------------------------------

std::string NullPlanner::getPathStats()
{
  std::vector<std::string> stats;

  std::string s{"algorithm=none"};
  stats.push_back(s);
  s = "planning_time=" + doubleToStringX(m_planning_end_time - m_planning_start_time, 2);
  stats.push_back(s);
  s = "path_len_to_go=" + doubleToStringX(m_path.length(), 2);
  stats.push_back(s);
  s = "path_len_traversed=" + doubleToStringX(m_path_len_traversed, 2);
  stats.push_back(s);

  return stringVectorToString(stats, ',');
}


bool NullPlanner::postPath()
{
  Notify(m_prefix + m_path_found_var, "true");
  Notify(m_prefix + m_path_stats_var, getPathStats());
  postFlags(m_traverse_flags);
  return (true);
}


//---------------------------------------------------------

std::string NullPlanner::printPlannerMode()
{
  switch (m_mode)
  {
  case PlannerMode::IDLE:
    return ("IDLE");
  case PlannerMode::REQUEST_PENDING:
    return ("REQUEST_PENDING");
  case PlannerMode::PLANNING_IN_PROGRESS:
    return ("PLANNING_IN_PROGRESS");
  case PlannerMode::PLANNING_FAILED:
    return ("PLANNING_FAILED");
  case PlannerMode::IN_TRANSIT:
    return ("IN_TRANSIT");
  case PlannerMode::PATH_COMPLETE:
    return ("PATH_COMPLETE");
  default:
    return ("UNKNOWN");
  }
}


void NullPlanner::postFlags(const std::vector<VarDataPair>& flags)
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

    sval = macroExpand(sval, "MODE", printPlannerMode());
    sval = macroExpand(sval, "PATH_SPEC", m_path.get_spec(2));
    sval = macroExpand(sval, "PATH_PTS", m_path.get_spec_pts(2));

    // if final val is a number, post as double
    if (isNumber(sval))
      Notify(moosvar, std::stod(sval));
    else
      Notify(moosvar, sval);
  }
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool NullPlanner::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  std::string grid_bounds{"pts="}, grid_cell_size{"cell_size="};

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    // vars to subscribe to
    if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, toupper(value));
    } else if (param == "obs_alert_var") {
      handled = setNonWhiteVarOnString(m_obs_alert_var, toupper(value));
    } else if (param == "wpt_complete_var") {
      handled = setNonWhiteVarOnString(m_wpt_complete_var, toupper(value));
    // publication config
    } else if (param == "prefix") {
      handled = setNonWhiteVarOnString(m_prefix, toupper(value));
    } else if ((param == "intermediate_pts") || param == "intermediate_points") {
      m_intermediate_pts = string2SegList(value);
      handled = (m_intermediate_pts.size() != 0);
    } else if ((param == "init_plan_flag") || param == "initflag") {
      handled = addVarDataPairOnString(m_init_plan_flags, value);
    } else if ((param == "traverse_flag") || param == "traverseflag") {
      handled = addVarDataPairOnString(m_traverse_flags, value);
    } else if ((param == "end_flag") || (param == "endflag")) {
      handled = addVarDataPairOnString(m_end_flags, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // fill in subscription variables with defaults if they weren't set in config
  // leave these variables empty in constructor so that we don't register for
  // variables we don't need
  if (m_path_request_var.empty()) m_path_request_var = "PLAN_PATH_REQUESTED";
  if (m_obs_alert_var.empty()) m_obs_alert_var = "OBSTACLE_ALERT";
  if (m_wpt_complete_var.empty()) m_wpt_complete_var = "WAYPOINTS_COMPLETE";

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void NullPlanner::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("OBM_RESOLVED", 0);
  Register("WPT_INDEX", 0);
  if (!m_path_request_var.empty()) Register(m_path_request_var, 0);
  if (!m_obs_alert_var.empty()) Register(m_obs_alert_var, 0);
  if (!m_wpt_complete_var.empty()) Register(m_wpt_complete_var, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool NullPlanner::buildReport()
{
  using std::endl;
  std::string header{"================================"};
  m_msgs << header << endl;
  m_msgs << "Subscriptions:" << endl;
  m_msgs << "  path_request_var: " << m_path_request_var << endl;
  m_msgs << "  obs_alert_var: " << m_obs_alert_var << endl;
  m_msgs << "  obs_resolved_var (not set by user): OBM_RESOLVED" << endl;
  m_msgs << "  wpt_complete_var: " << m_wpt_complete_var << endl;

  m_msgs << header << endl;
  m_msgs << "Flags:" << endl;
  m_msgs << "  Initial Plan Flags:" << endl;
  for (VarDataPair pair : m_init_plan_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  Traverse Flags:" << endl;
  for (VarDataPair pair : m_traverse_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  End Flags:" << endl;
  for (VarDataPair pair : m_end_flags)
    m_msgs << "    " << pair.getPrintable() << endl;

  m_msgs << header << endl;
  m_msgs << "Planner Mode: " << printPlannerMode() << endl;
  std::string start_spec{m_start_point.valid() ? m_start_point.get_spec() : "UNSET"};
  std::string goal_spec{m_goal_point.valid() ? m_goal_point.get_spec() : "UNSET"};
  m_msgs << "Start Point: " << start_spec << endl;
  m_msgs << "Goal Point: " << goal_spec << endl;
  if (!m_prefix.empty())
    m_msgs << "Publication Prefix: " << m_prefix << endl;

  m_msgs << header << endl;
  m_msgs << "Tracked Obstacles: " << endl;
  for (auto const& obs : m_obstacle_map)
    m_msgs << obs.first << ";";
  m_msgs << endl;

  m_msgs << header << endl;
  if (m_mode == PlannerMode::PLANNING_IN_PROGRESS) {
    m_msgs << "PLANNING IN PROGRESS" << endl;
    return (true);
  }

  m_msgs << "Path Stats:" << endl;
  m_msgs << "  Planning Time: " <<
    doubleToStringX(m_planning_end_time - m_planning_start_time, 2) << endl;
  m_msgs << "  Path Length: " << doubleToStringX(m_path.length(), 2) << endl;
  m_msgs << "  Path Length Previously Traversed: " <<
    doubleToStringX(m_path_len_traversed, 2) << endl;
  return(true);
}
