/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: LPAStar.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iostream>
#include <iterator>
#include <map>
#include <string>
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
#include "XYGridUpdate.h"
#include "VarDataPair.h"
#include "VarDataPairUtils.h"


//---------------------------------------------------------
// Constructor()

LPAStar::LPAStar()
{
  m_path_request_var = "";  // set in onStartup
  m_obs_alert_var = "";  // set in onStartup
  m_wpt_complete_var = "";  // set in onStartup

  m_prefix = "";
  m_path_found_var = "PATH_FOUND";
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
      handleObstacleResolved(msg.GetString());
    } else if (key == m_wpt_complete_var) {
      if (m_mode == PlannerMode::IN_TRANSIT)
        m_mode = PlannerMode::PATH_COMPLETE;
    } else if ((key == "NODE_REPORT_LOCAL") || (key == "NODE_REPORT")) {
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

  // add new obstacle to the ADD queue
  std::string key{new_obs.get_label()};
  m_obstacle_add_queue[key] = new_obs;

  // if this is an obstacle with a label we've seen before,
  // add it to the REFRESH queue so we can remove the old
  // obstacle in the iterate loop
  // REFRESH queue is a subset of the ADD queue
  if (m_obstacle_map.count(key))
    m_obstacle_refresh_queue.insert(key);

  reportEvent("new obstacle: " + key);
  return (true);
}


bool LPAStar::handleObstacleResolved(std::string obs_label)
{
  // if obs is in ADD/REFRESH queue for some reason, remove it
  m_obstacle_add_queue.erase(obs_label);
  m_obstacle_refresh_queue.erase(obs_label);

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
  if (m_mode != PlannerMode::PLANNING_IN_PROGRESS)
    syncObstacles();

  bool path_found{false};

  // new path request
  if (m_mode == PlannerMode::REQUEST_PENDING) {
    m_mode = PlannerMode::PLANNING_IN_PROGRESS;

    // post new plan messages
    Notify(m_prefix + m_path_found_var, "false");
    Notify(m_prefix + m_path_complete_var, "false");
    postFlags(m_init_plan_flags);

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
    if (!checkObstacles()) {
      //! temporarily disable
      // Notify(m_prefix + m_path_found_var, "false");
      // postFlags(m_replan_flags);

      // // plan path until we reach max number of iterations
      // m_mode = PlannerMode::PLANNING_IN_PROGRESS;
      // m_planning_start_time = MOOSTime();
      // path_found = replanFromCurrentPos();  // todo: what if this takes multiple iterations?
    }
  }

  // notify that path has been found
  if (path_found) {
    m_planning_end_time = MOOSTime();
    m_mode = PlannerMode::IN_TRANSIT;
    postPath();  // todo: when to post previous path stats?
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
// Generic Procedures

bool LPAStar::planPath()
{
  if (!checkPlanningPreconditions()) {
    m_mode = PlannerMode::PLANNING_FAILED;
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
  Notify(m_prefix + m_path_found_var, "true");
  Notify(m_prefix + "PATH_STATS", getPathStats());
  postFlags(m_traverse_flags);
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


std::string LPAStar::printPlannerMode()
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


void LPAStar::postFlags(const std::vector<VarDataPair>& flags)
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
    sval = macroExpand(sval, "START_X", m_start_point.get_vx(), 2);
    sval = macroExpand(sval, "START_Y", m_start_point.get_vy(), 2);
    sval = macroExpand(sval, "GOAL_X", m_goal_point.get_vx(), 2);
    sval = macroExpand(sval, "GOAL_Y", m_goal_point.get_vy(), 2);
    sval = macroExpand(sval, "V_X", m_vpos.get_vx(), 2);
    sval = macroExpand(sval, "V_Y", m_vpos.get_vy(), 2);

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
// LPA* Procedures

bool LPAStar::checkPlanningPreconditions()
{
  std::vector<std::string> warnings;

  // check all preconditions
  if (m_grid.size() == 0)
    warnings.push_back("Grid configured incorrectly!");
  if (m_max_iters <= 0)
    warnings.push_back("Iteration limit <= 0, must be positive!");
  if (!m_start_point.valid() || !m_goal_point.valid())
    warnings.push_back("Start or goal point not valid!");
  if (!m_grid.ptIntersect(m_start_point.get_vx(), m_start_point.get_vy()))
    warnings.push_back("Start point located outside search grid!");
  if (!m_grid.ptIntersect(m_goal_point.get_vx(), m_goal_point.get_vy()))
    warnings.push_back("Goal point located outside search grid!");

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


// cells with obstacles in them are marked impassible
void LPAStar::syncObstacles()
{
  // no obstacles to change, exit early
  if ((m_obstacle_add_queue.empty()) && (m_obstacle_remove_queue.empty()))
    return;

  XYGridUpdate update{m_grid.get_label()};
  update.setUpdateTypeReplace();

  // update obstacle grid
  unsigned int cix{m_grid.getCellVarIX("obs")};
  for (int ix = 0; ix < m_grid.size(); ix++) {
    XYSquare grid_cell{m_grid.getElement(ix)};
    bool cell_clear_pending{false};

    // if grid cell intersects with an existing obstacle that needs to be
    // updated, mark the cell to be cleared
    for (auto const& obs : m_obstacle_refresh_queue) {
      if (m_obstacle_map[obs].intersects(grid_cell)) {
        cell_clear_pending = true;
        break;  // only need to check for a single intersection
      }
    }

    // if grid cell intersects with an obstacle that is marked to remove,
    // mark the cell to be cleared
    for (auto const& obs : m_obstacle_remove_queue) {
      if (m_obstacle_map[obs].intersects(grid_cell)) {
        cell_clear_pending = true;
        break;  // only need to check for a single intersection
      }
    }

    // if cell is marked to be cleared, check if it intersects with any
    // obstacle that should NOT be removed/refreshed.
    // if so, don't clear the cell
    if (cell_clear_pending) {
      for (auto const& obs : m_obstacle_map) {
        // if obstacle is on REFRESH/REMOVED queue, we don't need to check again
        if (m_obstacle_refresh_queue.count(obs.first))
          continue;
        if (m_obstacle_remove_queue.count(obs.first))
          continue;

        // does the cell intersect with any obstacles that should be unchanged?
        if (obs.second.intersects(m_grid.getElement(ix))) {
          cell_clear_pending = false;
          break;
        }
      }
    }
    // we're good to clear the cell, go ahead
    if (cell_clear_pending) {
      m_grid.setVal(ix, 0, cix);
      update.addUpdate(ix, "obs", 0);
    }

    // if grid cell intersects with an obstacle to add, set cell to 1
    for (auto const& obs : m_obstacle_add_queue) {
      if (obs.second.intersects(grid_cell)) {
        m_grid.setVal(ix, 1, cix);
        update.addUpdate(ix, "obs", 1);
        break;  // only need to check for a single intersection
      }
    }
  }

  // update obstacle map and clear queues
  for (auto const& obs : m_obstacle_remove_queue)
    m_obstacle_map.erase(obs);
  for (auto const& obs : m_obstacle_add_queue)
    m_obstacle_map[obs.first] = obs.second;
  m_obstacle_add_queue.clear();
  m_obstacle_refresh_queue.clear();
  m_obstacle_remove_queue.clear();

  // post visuals
  if ((m_post_visuals) && (update.size() > 0))
    Notify("VIEW_GRID_DELTA", update.get_spec());
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
    } else if (param == "init_plan_flag") {
      handled = addVarDataPairOnString(m_init_plan_flags, value);
    } else if (param == "traverse_flag") {
      handled = addVarDataPairOnString(m_traverse_flags, value);
    } else if (param == "replan_flag") {
      handled = addVarDataPairOnString(m_replan_flags, value);
    } else if ((param == "end_flag") || (param == "endflag")) {
      handled = addVarDataPairOnString(m_end_flags, value);
    } else if (param == "post_visuals") {
      handled = setBooleanOnString(m_post_visuals, value);
    // planning config
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

  // create grid based on parameters
  std::string cell_vars{"cell_vars=obs:0:vertex:0"};
  std::string obs_min_max{"cell_min=obs:0, cell_max=obs:1"};
  std::string grid_config{grid_bounds + "," + grid_cell_size + "," + cell_vars + "," + obs_min_max};
  m_grid = string2ConvexGrid(grid_config);
  if (m_grid.size() == 0)
    reportConfigWarning("Unable to generate grid for LPA* algorithm due to bad config!");

  // post grid visuals
  m_grid.set_label("LPA*");
  if (m_post_visuals)
    Notify("VIEW_GRID", m_grid.get_spec());

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void LPAStar::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
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
  std::string header{"================================"};
  m_msgs << header << endl;
  m_msgs << "Subscriptions:" << endl;
  m_msgs << "  path_request_var: " << m_path_request_var << endl;
  m_msgs << "  obs_alert_var: " << m_obs_alert_var << endl;
  m_msgs << "  obs_resolved_var (not set by user): OBM_RESOLVED" << endl;
  m_msgs << "  wpt_complete_var: " << m_wpt_complete_var << endl;

  m_msgs << header << endl;
  m_msgs << "Publications:" << endl;
  m_msgs << "  " << m_prefix << m_path_found_var << endl;
  m_msgs << "  " << m_prefix << m_path_complete_var << endl;
  m_msgs << header << endl;
  m_msgs << "Flags:" << endl;
  m_msgs << "  Initial Plan Flags:" << endl;
  for (VarDataPair pair : m_init_plan_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  Traverse Flags:" << endl;
  for (VarDataPair pair : m_traverse_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  Replan Flags:" << endl;
  for (VarDataPair pair : m_replan_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  End Flags:" << endl;
  for (VarDataPair pair : m_end_flags)
    m_msgs << "    " << pair.getPrintable() << endl;

  m_msgs << header << endl;
  m_msgs << "Planner Mode: " << printPlannerMode() << endl;
  m_msgs << "Grid Config: " << m_grid.getConfigStr() << endl;
  std::string start_spec{m_start_point.valid() ? m_start_point.get_spec() : "UNSET"};
  std::string goal_spec{m_goal_point.valid() ? m_goal_point.get_spec() : "UNSET"};
  m_msgs << "Start Point: " << start_spec << endl;
  m_msgs << "Goal Point: " << goal_spec << endl;

  m_msgs << header << endl;
  m_msgs << "Tracked Obstacles: ";
  for (auto const& obs : m_obstacle_map)
    m_msgs << obs.first << ";";
  m_msgs << endl;

  m_msgs << header << endl;
  m_msgs << "Path Stats:" << "todo" << endl;
  return(true);
}
