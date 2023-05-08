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
#include "MacroUtils.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "LPAStar.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "XYSquare.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsConvexGrid.h"
#include "VarDataPair.h"


//---------------------------------------------------------
// Constructor()

LPAStar::LPAStar()
{
  m_path_request_var = "";  // set in onStartup
  m_obs_alert_var = "";  // set in onStartup
  m_path_found_var = "PATH_FOUND";
  m_wpt_update_var = "PATH_UPDATE";
  m_wpt_complete_var = "";  // set in onStartup
  m_path_complete_var = "PATH_COMPLETE";

  m_max_iters = 1000;
  m_post_visuals = true;

  m_mode = PlannerMode::IDLE;
  m_planning_start_time = 0;
  m_planning_end_time = 0;

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
        m_mode = PlannerMode::REQUEST_PENDING;
      else
        reportRunWarning("Invalid " + key + ": " + msg.GetString());
    } else if (key == m_obs_alert_var) {
      handleObstacleAlert(msg.GetString());
    } else if (key == "OBM_RESOLVED") {
      std::string key{msg.GetString()};
      if (m_obstacle_map.count(key))
        m_obstacle_map.erase(key);
    } else if (key == m_wpt_complete_var) {
      if (m_mode == PlannerMode::IN_TRANSIT)
        m_mode = PlannerMode::PATH_COMPLETE;
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

  // apply spec to start and goal, if parsing fails return fail
  m_start_point = string2Point(start);
  m_goal_point = string2Point(goal);
  if (!m_start_point.valid() || !m_goal_point.valid())
    return (false);

  return (true);
}


bool LPAStar::handleObstacleAlert(std::string obs_alert)
{
  // name=avd_obstacles_ob_0#poly=pts={-21.69,-135.31:-24.25,-132.75:-10.83,-132.03},label=ob_0
  // pull out polygon from alert message
  biteStringX(obs_alert, '#');  // removes name block
  biteStringX(obs_alert, '=');  // removes poly= block, just leaves pts=

  // parse obstacle from string
  XYPolygon new_obs{string2Poly(obs_alert)};
  if (!new_obs.is_convex())
    return(false);

  // add obstacle to obstacle map
  std::string key{new_obs.get_label()};
  m_obstacle_map[key] = new_obs;

  reportEvent("new obstacle: " + key);
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

  // refresh obstacle grid each iteration if we're not
  // actively planning
  if ((m_mode != PlannerMode::IDLE) && (m_mode != PlannerMode::PLANNING_IN_PROGRESS)) {
    m_grid.reset("obs");
    addObsToGrid();
  }


  bool path_found{false};

  // new path request
  if (m_mode == PlannerMode::REQUEST_PENDING) {
    m_mode = PlannerMode::PLANNING_IN_PROGRESS;

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
    m_planning_start_time = MOOSTime();
    m_path.clear();  // start fresh upon new request
    path_found = planPath();

  // if already planning a path, pick up from where we left off
  } else if (m_mode == PlannerMode::PLANNING_IN_PROGRESS) {
    //! path_found = planPath();  temporarily disable
    path_found = true;

  // if transiting, check if we need to replan and replan if needed
  } else if (m_mode == PlannerMode::IN_TRANSIT) {
    //! temporarily disable
    // if (!checkObstacles()) {
    //   Notify(m_path_found_var, "false");
    //   Notify("STATION_UPDATES", "center_activate=true");  // todo: add as replan flag

    //   // plan path until we reach max number of iterations
    //   m_mode = PlannerMode::PLANNING_IN_PROGRESS;
    //   m_planning_start_time = MOOSTime();
    //   path_found = replanFromCurrentPos();  // todo: what if this takes multiple iterations?
    // }
  }

  // notify that path has been found
  if (path_found) {
    m_planning_end_time = MOOSTime();
    m_mode = PlannerMode::IN_TRANSIT;
    postPath();  // todo: when to post previous path stats?
  }

  // if path is complete, cleanup
  if (m_mode == PlannerMode::PATH_COMPLETE) {
    // todo: post endflags
    Notify(m_path_complete_var, "true");
    m_mode = PlannerMode::IDLE;
  }

  // if we're not idle, post visuals
  if ((m_post_visuals)  && (m_mode != PlannerMode::IDLE))
    postGrid();

  AppCastingMOOSApp::PostReport();
  return (true);
}


//---------------------------------------------------------
// Generic Procedures

bool LPAStar::planPath()
{
  if (!checkPlanningPreconditions()) {
    m_mode = PlannerMode::PLANNING_FAILED;
    reportEvent("Cannot plan because " + GetAppName() + " is configured incorrectly.");
    return (false);
  }

  // placeholder: path is just start point and goal point
  m_path.add_vertex(m_start_point);
  m_path.add_vertex(m_goal_point);
  // todo TJ: remove add vertex lines and implement LPA* here

  return (true);  // todo: instead of returning true, set mode to PLAN_IN_PROGRESS or PLAN_FAILED
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
  // grab pairs of points on path
  for (int i = 0; i < m_path.size() - 1; i++) {
    double x1{m_path.get_vx(i)}, y1{m_path.get_vy(i)};
    double x2{m_path.get_vx(i + 1)}, y2{m_path.get_vy(i + 1)};

    // if line between points intersects an obstacle, replan false
    for (auto const& obs : m_obstacle_map) {
      if (obs.second.seg_intercepts(x1, y1, x2, y2)) {
        reportEvent("Replan required, path intersects with " + obs.first);
        return (false);
      }
    }
  }
  return (true);
}


bool LPAStar::replanFromCurrentPos()
{
  // todo Raul: define a skeleton, give to TJ
  return (true);
}


//---------------------------------------------------------
// LPA* Procedures

bool LPAStar::checkPlanningPreconditions()
{
  // todo: add run warnings for each of these
  if (m_grid.size() == 0)
    return (false);
  if (m_max_iters <= 0)
    return (false);
  if (!m_start_point.valid() || !m_goal_point.valid())
    return (false);
  if (!m_grid.ptIntersect(m_start_point.get_vx(), m_start_point.get_vy()))
    return (false);
  if (!m_grid.ptIntersect(m_goal_point.get_vx(), m_goal_point.get_vy()))
    return (false);

  return (true);
}


// cells with obstacles in them are marked impassible
void LPAStar::addObsToGrid()
{
  unsigned int cix{m_grid.getCellVarIX("obs")};

  for (int ix = 0; ix < m_grid.size(); ix++) {
    // for each obstacle, if obs intersects with grid cell, set val to 1
    for (auto const& obs : m_obstacle_map) {
      if (obs.second.intersects(m_grid.getElement(ix))) {
        m_grid.setVal(ix, 1, cix);
        break;  // if one obs intersects we don't need to check the rest
      }
    }
  }
}


bool LPAStar::postGrid()
{
  // todo: implement this, in which system modes?
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

  std::string grid_bounds{"pts="}, grid_cell_size{"cell_size="};

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
    } else if (param == "grid_bounds") {
      if (!strBegins(value, "{"))
        value = "{" + value;
      if (!strEnds(value, "}"))
        value += "}";
      grid_bounds += value;
      handled = true;
    } else if (param == "grid_cell_size") {
      grid_cell_size += value;
      handled = true;
    } else if (param == "max_planning_iters") {
      int max_iters{std::stoi(value)};
      if (max_iters > 0) {
        handled = true;
        m_max_iters = max_iters;
      } else {
        reportConfigWarning("Max iterations must be above 0, received " + value);
      }
    } else if (param == "post_visuals") {
      handled = setBooleanOnString(m_post_visuals, value);
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

  // create grid based on parameters
  std::string cell_vars{"cell_vars=obs:0:vertex:0"};
  m_grid = string2ConvexGrid(grid_bounds + "," + grid_cell_size + "," + cell_vars);
  if (m_grid.size() == 0)
    reportConfigWarning("Unable to generate grid for LPA* algorithm due to bad config!");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void LPAStar::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT_LOCAL", 0);
  Register("OBM_RESOLVED", 0);
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
  // todo: add flags here, add planner mode, add obstacles, add vars, add grid bounds/cell size
  return(true);
}
