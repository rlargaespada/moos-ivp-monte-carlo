/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: LPAStar.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include "MBUtils.h"
#include "ACTable.h"
#include "LPAStar.h"
#include "XYPoint.h"


//---------------------------------------------------------
// Constructor()

LPAStar::LPAStar()
{
  m_path_request_var = "";
  m_obs_alert_var = "";
  m_path_found_var = "PATH_FOUND";
  m_wpt_update_var = "PATH_UPDATE";
  m_wpt_complete_var = "";
  m_path_complete_var = "PATH_COMPLETE";

  m_grid_density = 2;  // meters

  m_path_request_pending = false;
  m_planning_start_time = 0;
  m_planning_in_progress = false;
  m_transiting = false;
  m_path_complete = false;

  m_start_point.invalidate();
  m_goal_point.invalidate();
}

//---------------------------------------------------------
// Destructor

LPAStar::~LPAStar()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool LPAStar::OnNewMail(MOOSMSG_LIST &NewMail)
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
      if (setEndpoints(msg.GetString()))
        m_path_request_pending = true;
      else
        reportRunWarning("Invalid " + key + ": " + msg.GetString());
    } else if (key == "GIVEN_OBSTACLE") {
      // todo: add to obstacle vec
    } else if (key == m_obs_alert_var) {
      // todo: add to obstacle vec
    } else if (key == m_wpt_complete_var) {
      if (m_transiting) {
        m_transiting = false;
        m_path_complete = true;
      }
      // todo: post endflags
    } else if (key == "NODE_REPORT_LOCAL") {
      std::string report{tolower(msg.GetString())};
      double xval{tokDoubleParse(report, "x", ',', '=')};
      double yval{tokDoubleParse(report, "y", ',', '=')};
      m_vpos.set_vertex(xval, yval);
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return(true);
}


bool LPAStar::setEndpoints(std::string request)
{
  std::string start, goal;
  request = tolower(request);

  // pull out start and goal, return fail if can't find
  start = tokStringParse(request, "start", ';', '=');
  goal = tokStringParse(request, "goal", ';', '=');
  if ((start.empty()) || (goal.empty()))
    return (false);

  // parse start pos x and y, return fail if not found, otherwise set vertex
  std::string start_x, start_y;
  start_x = biteStringX(start, ',');
  start_y = start;  // remainder is just y value
  if ((start_x.empty()) || (start_y.empty()))
    return (false);
  m_start_point.set_vertex(std::stod(start_x), std::stod(start_y));

  // parse goal pos x and y, return fail if not found, otherwise set vertex
  std::string goal_x, goal_y;
  goal_x = biteStringX(goal, ',');
  goal_y = goal;  // remainder is just y value
  if ((goal_x.empty()) || (goal_y.empty()))
    return (false);
  m_goal_point.set_vertex(std::stod(goal_x), std::stod(goal_y));

  return (true);
}


//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool LPAStar::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool LPAStar::Iterate()
{
  AppCastingMOOSApp::Iterate();

  bool path_found{false};

  // new path request
  if (m_path_request_pending) {
    m_path_request_pending = false;

    // post new plan messages
    Notify(m_path_found_var, "false");
    Notify(m_path_complete_var, "false");
    // todo: STATION_UPDATES is hardcoded for now but should be configured as a flag w/ macros
    std::string station_pt{doubleToStringX(m_start_point.get_vx(), 2)};
    station_pt += "," + doubleToStringX(m_start_point.get_vy(), 2);
    Notify("STATION_UPDATES", "station_pt=" + station_pt + "# center_activate=false");

    // todo: what if this takes a few iterations? need to break into steps
    // todo: what if planning fails?
    // plan path until we reach max number of iterations
    m_planning_in_progress = true;
    m_transiting = false;
    m_path_complete = false;
    m_planning_start_time = MOOSTime();
    path_found = planPath();

  // if already planning a path, pick up from where we left off
  } else if (m_planning_in_progress) {
    m_planning_in_progress = false;
    // path_found = planPath();  temporarily disable

  // if transiting, check if we need to replan and replan if needed
  } else if (m_transiting) {
    if (!checkObstacles()) {
      Notify(m_path_found_var, "false");
      Notify("STATION_UPDATES", "center_activate=true");  // todo: add as replan flag

      // plan path until we reach max number of iterations
      m_planning_start_time = MOOSTime();
      m_planning_in_progress = true;
      m_transiting = false;
      path_found = replanFromCurrentPos();  // todo: what if this takes multiple iterations?
    }
  }

  // notify that path has been found
  if (path_found) {
    m_planning_end_time = MOOSTime();
    m_planning_in_progress = false;
    postPath();  // todo: when to post previous path stats?
    m_transiting = true;
  }

  if (m_path_complete) {
    // todo: post endflags
    Notify(m_path_complete_var, "true");
    m_path_complete = false;
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}


//---------------------------------------------------------
// Generic Procedures

bool LPAStar::planPath()
{
  // clear previous state, add current set of obstacles
  // todo: these should only be called for first iteration
  m_path.clear();
  clearGrid();
  addObsToGrid();

  // placeholder: path is just start point and goal point
  m_path.add_vertex(m_start_point);
  m_path.add_vertex(m_goal_point);
  // todo TJ: remove add vertex lines and implement LPA* here

  return (true);
}


std::string LPAStar::getPathStats()
{
  // todo Raul: implement this function
  return ("");
}


bool LPAStar::postPath()
{
  Notify(m_path_found_var, "true");
  Notify(m_wpt_update_var, "points = " + m_path.get_spec_pts(2));
  Notify("PATH_STATS", getPathStats());
  // todo Raul: add traverse flags
  return (true);
}


bool LPAStar::checkObstacles()
{
  // todo TJ: implement this function
  return (true);
}


bool LPAStar::replanFromCurrentPos()
{
  // todo Raul: define a skeleton, give to TJ
  return (true);
}


//---------------------------------------------------------
// LPA* Procedures
// sets all cells in grid to empty
void LPAStar::clearGrid()
{
  // todo TJ: implement this function
}


// cells with obstacles in them are marked impassible
bool LPAStar::addObsToGrid()
{
  // todo TJ: implement this function
  return (true);
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool LPAStar::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    // todo: pick which vars should be hardcoded, which vars should be configurable
    // todo: add prefix config var, so PATH_* -> <prefix>_*
    if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, value);
    } else if (param == "obs_alert_var") {
      handled = setNonWhiteVarOnString(m_obs_alert_var, value);
    } else if (param == "path_found_var") {
      handled = setNonWhiteVarOnString(m_path_found_var, value);
    } else if (param == "wpt_update_var") {
      handled = setNonWhiteVarOnString(m_wpt_update_var, value);
    } else if (param == "wpt_complete_var") {
      handled = setNonWhiteVarOnString(m_wpt_complete_var, value);
    } else if (param == "path_complete_var") {
      handled = setNonWhiteVarOnString(m_path_complete_var, value);
    }
    // todo: add initial plan flags, replanflags, traverseflags, endflags,
    // macro should include start and goal

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // fill in subscription variables with defaults if they weren't set in config
  // leave these variables empty in constructor so that we don't register for
  // variables we don't need
  if (m_path_request_var.empty())
    m_path_request_var = "PLAN_PATH_REQUESTED";
  if (m_obs_alert_var.empty())
    m_obs_alert_var = "OBSTACLE_ALERT";
  if (m_wpt_complete_var.empty())
    m_wpt_complete_var = "WAYPOINTS_COMPLETE";

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void LPAStar::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("GIVEN_OBSTACLE", 0);
  Register("NODE_REPORT_LOCAL", 0);
  if (!m_path_request_var.empty())
    Register(m_path_request_var, 0);
  if (!m_obs_alert_var.empty())
    Register(m_obs_alert_var, 0);
  if (!m_wpt_complete_var.empty())
    Register(m_wpt_complete_var, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool LPAStar::buildReport()
{
  using std::endl;

  std::string start_spec{m_start_point.valid() ? m_start_point.get_spec() : "UNSET"};
  std::string goal_spec{m_goal_point.valid() ? m_goal_point.get_spec() : "UNSET"};
  m_msgs << "Start Point: " << start_spec << endl;
  m_msgs << "Goal Point: " << goal_spec << endl;
  m_msgs << "Vehicle Position: "  << m_vpos.get_spec() << endl;
  // todo: add flags here
  return(true);
}
