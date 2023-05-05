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


//---------------------------------------------------------
// Constructor()

LPAStar::LPAStar()
{
  m_path_found_var = "PATH_FOUND";
  m_wpt_update_var = "PATH_UPDATE";
  m_path_complete_var = "PATH_COMPLETE";

  m_grid_density = 2;  // meters

  m_path_request_pending = false;
  m_transiting = false;
  m_replan_needed = false;

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
      // todo: post endflags
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return(true);
}


bool LPAStar::setEndpoints(std::string request)
{
  std::string start, goal, xval, yval;
  request = tolower(request);

  // pull out start and goal, return fail if can't find
  start = tokStringParse(request, "start", ';', '=');
  goal = tokStringParse(request, "goal", ';', '=');
  if ((start.empty()) || (goal.empty()))
    return (false);

  // parse start pos x and y, return fail if not found, otherwise set vertex
  xval = biteStringX(start, ',');
  yval = start;  // remainder is just y value
  if ((xval.empty()) || (yval.empty()))
    return (false);
  m_start_point.set_vertex(std::stod(xval), std::stod(yval));

  xval.clear();
  yval.clear();

  // parse goal pos x and y, return fail if not found, otherwise set vertex
  xval = biteStringX(goal, ',');
  yval = goal;  // remainder is just y value
  if ((xval.empty()) || (yval.empty()))
    return (false);
  m_goal_point.set_vertex(std::stod(xval), std::stod(yval));

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

  if (m_path_request_pending) {
    double start_time{MOOSTime()};
    planPath();
    postPath();
    double elapsed_time{MOOSTime() - start_time};
    Notify("PLANNING_TIME", elapsed_time);
  } else if (m_transiting) {
    checkObstacles();
    if (m_replan_needed) {
      double start_time{MOOSTime()};
      replanFromCurrentPos();
      postPath();  // todo: when to post previous path stats?
      double elapsed_time{MOOSTime() - start_time};
      Notify("PLANNING_TIME", elapsed_time);
    }
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}


//---------------------------------------------------------
// Generic Procedures

bool LPAStar::planPath()
{}


double LPAStar::getPathLength()
{}


std::string LPAStar::getPathSpec()
{}


bool LPAStar::postPath()
{}


bool LPAStar::checkObstacles()
{}


bool LPAStar::replanFromCurrentPos()
{}


//---------------------------------------------------------
// LPA* Procedures
bool LPAStar::clearGrid()
{}


bool LPAStar::addObsToGrid()
{}


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
  return(true);
}
