/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DStarLite.cpp                                        */
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
#include "ACTable.h"
#include "DStarLite.h"
#include "MacroUtils.h"
#include "MBUtils.h"
#include "VarDataPair.h"
#include "VarDataPairUtils.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsConvexGrid.h"
#include "XYGridUpdate.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "XYSquare.h"


//---------------------------------------------------------
// Constructor()

DStarLite::DStarLite()
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
  m_post_visuals = true;

  // D* Lite config
  m_max_iters = 1000;

  //* State Variables
  m_start_cell = -1;  // set these to -1 to signify they're invalid
  m_goal_cell = -1;
  m_vpos_cell = -1;

  // planning state data
  m_mode = PlannerMode::IDLE;
  m_planning_start_time = 0;
  m_planning_end_time = 0;

  // D* Lite state
  m_last_cell = -1;  // -1 is invalid
  m_k_m = 0;
}


//---------------------------------------------------------
// Destructor

DStarLite::~DStarLite()
{
}


//---------------------------------------------------------
// Procedure: OnNewMail()

bool DStarLite::OnNewMail(MOOSMSG_LIST &NewMail)
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


bool DStarLite::setEndpoints(std::string request)
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


bool DStarLite::handleObstacleAlert(std::string obs_alert)
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


bool DStarLite::handleObstacleResolved(const std::string obs_label)
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

bool DStarLite::OnConnectToServer()
{
  registerVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool DStarLite::Iterate()
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

bool DStarLite::checkPlanningPreconditions()
{
  std::vector<std::string> warnings;

  // check all preconditions
  if (m_grid.size() == 0)
    warnings.push_back("Grid configured incorrectly!");
  if (m_max_iters <= 0)
    warnings.push_back("Iteration limit <= 0, must be positive!");
  if (!m_start_point.valid() || !m_goal_point.valid())
    warnings.push_back("Start or goal point not valid!");
  if (!m_grid.ptIntersect(m_start_point.x(), m_start_point.y()))
    warnings.push_back("Start point located outside search grid!");
  if (!m_grid.ptIntersect(m_goal_point.x(), m_goal_point.y()))
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
void DStarLite::syncObstacles()
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


bool DStarLite::planPath()
{
  if (!checkPlanningPreconditions()) {
    m_mode = PlannerMode::PLANNING_FAILED;
    return (false);
  }

  // placeholder: path is just start point and goal point
  m_path.add_vertex(m_start_point);
  m_path.add_vertex(m_goal_point);
  // todo TJ: remove add vertex lines and implement D* Lite here

  /*
  initialize
  planning done = compute shortest path
  if planning_done
    raul: parse path out by following grid?
    if can't parse path, set mode to planning failed, report event, return false
    otherwise set path to m_path and return true
  else planning's not done, so continue from where we left off without initializing
  */

  return (true);
}


//---------------------------------------------------------

std::set<int> DStarLite::getNeighbors(int grid_ix)
{
  if (m_neighbors.count(grid_ix))  // check in cache first
    return (m_neighbors[grid_ix]);

  // center of search cell + cell size
  double cx{m_grid.getElement(grid_ix).getCenterX()};
  double cy{m_grid.getElement(grid_ix).getCenterY()};
  double l{m_grid.getCellSize()};

  // loop variables
  int neighbor_ix{grid_ix - 1};
  int center_ix{0};
  XYSquare possible_neighbor;
  double neighbor_cx, neighbor_cy;

  // find neighbors with grid indices less than index of search cell
  // neighbors of search cell should have centers at the following points
  std::vector<std::pair<double, double>> neighbor_centers{
    {cx, cy - l},  // south
    {cx - l, cy + l},  // northeast
    {cx - l, cy},  // east
    {cx - l, cy - l},  // southeast
  };

  // search for neigbors starting with the index of the search cell - 1
  while ((neighbor_ix > 0) && (center_ix < neighbor_centers.size())) {
    neighbor_cx = neighbor_centers[center_ix].first;
    neighbor_cy = neighbor_centers[center_ix].second;

    // neighbor center is out of bounds, skip
    if (!m_grid_bounds.contains(neighbor_cx, neighbor_cy)) {
      center_ix += 1;
      continue;
    }

    // if neighbor center is contained in the cell at neighbor_ix,
    // then it's a neighbor of the search cell; save it and continue
    possible_neighbor = m_grid.getElement(neighbor_ix);
    if (possible_neighbor.containsPoint(neighbor_cx, neighbor_cy)) {
      m_neighbors[grid_ix].insert(neighbor_ix);
      neighbor_ix -= 1;
      center_ix += 1;
      continue;
    } else {
      neighbor_ix -= 1;
    }
  }

  neighbor_ix = grid_ix + 1;
  center_ix = 0;

  // find neighbors with grid indices greater than index of search cell
  // neighbors of search cell should have centers at the following points
  neighbor_centers = {
    {cx, cy + l},  // north
    {cx + l, cy - l},  // southwest
    {cx + l, cy},  // west
    {cx + l, cy + l}  // northwest
  };

  // search for neigbors starting with the index of the search cell + 1
  while ((neighbor_ix < m_grid.size()) && (center_ix < neighbor_centers.size())) {
    neighbor_cx = neighbor_centers[center_ix].first;
    neighbor_cy = neighbor_centers[center_ix].second;

    // neighbor center is out of bounds, skip
    if (!m_grid_bounds.contains(neighbor_cx, neighbor_cy)) {
      center_ix += 1;
      continue;
    }

    // if neighbor center is contained in the cell at neighbor_ix,
    // then it's a neighbor of the search cell; save it and continue
    possible_neighbor = m_grid.getElement(neighbor_ix);
    if (possible_neighbor.containsPoint(neighbor_cx, neighbor_cy)) {
      m_neighbors[grid_ix].insert(neighbor_ix);
      neighbor_ix += 1;
      center_ix += 1;
      continue;
    } else {
      neighbor_ix += 1;
    }
  }

  return (m_neighbors[grid_ix]);
}


double DStarLite::heuristic(int cell1, int cell2)
{
  double x1{m_grid.getElement(cell1).getCenterX()};
  double y1{m_grid.getElement(cell1).getCenterY()};
  double x2{m_grid.getElement(cell2).getCenterX()};
  double y2{m_grid.getElement(cell2).getCenterY()};
  return (hypot(x2 - x1, y2 - y2));
}


dsl_key DStarLite::calculateKey(int grid_ix)
{
  unsigned int g_cix{m_grid.getCellVarIX("g")}, rhs_cix{m_grid.getCellVarIX("rhs")};
  double g{m_grid.getVal(grid_ix, g_cix)}, rhs{m_grid.getVal(grid_ix, rhs_cix)};
  double first{std::min(g, rhs) + heuristic(m_start_cell, grid_ix) + m_k_m};
  double second{std::min(g, rhs)};
  return (dsl_key {first, second});
}


void DStarLite::initializeDStarLite()
{
  // clear priority queue, k_m
  m_dstar_queue.clear();
  m_k_m = 0;

  // initialize g and rhs values
  // todo: do this as we go instead of all at once
  unsigned int g_cix{m_grid.getCellVarIX("g")}, rhs_cix{m_grid.getCellVarIX("rhs")};
  for (int ix = 0; ix < m_grid.size(); ix++) {
    m_grid.setVal(ix, INFINITY, g_cix);
    m_grid.setVal(ix, INFINITY, rhs_cix);
  }

  // add goal state to priority queue
  m_grid.setVal(m_goal_cell, 0, rhs_cix);
  m_dstar_queue[m_goal_cell] = dsl_key{heuristic(m_start_cell, m_goal_cell), 0};
}


void DStarLite::updateVertex(int grid_ix)
{
  // if grid_ix != start
    // set rhs(ix)
  // if ix in queue, remove
  // if ix.g != ix.rhs, add to queue
}


bool DStarLite::computeShortestPath(int max_iters)
{
  /*
  iters = 0
  while (queue.top < calcKey(goal) || goal.rhs != goal.g)
    node = queue.pop
    if (node.g > node.rhs)
      node.g = node.rhs
      for all s in getNeighbors(node)
        Update vertex(s)
    else
      node.g = INFINITY
      for all s in getNeighbors(node)
        Update vertex(s)
      Update vertex(node)  //? required?

      if iters > m_max_iters
        return false;  // didn't find a path within allotted iterations

  return true  // found a path
  */

  return (true);
}


//---------------------------------------------------------

std::string DStarLite::getPathStats()
{
  // todo Raul: implement this function
  return ("");
}


bool DStarLite::postPath()
{
  Notify(m_prefix + m_path_found_var, "true");
  Notify(m_prefix + "PATH_STATS", getPathStats());
  postFlags(m_traverse_flags);
  return (true);
}


//---------------------------------------------------------

bool DStarLite::checkObstacles()
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


bool DStarLite::replanFromCurrentPos()
{
  // todo Raul: define a skeleton, give to TJ
  return (true);
}


//---------------------------------------------------------

std::string DStarLite::printPlannerMode()
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


void DStarLite::postFlags(const std::vector<VarDataPair>& flags)
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

bool DStarLite::OnStartUp()
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
  m_grid_bounds = string2Poly(grid_bounds);
  std::string cell_vars{"cell_vars=obs:0:g:0:rhs:0"};  // add vars for D* Lite
  std::string obs_min_max{"cell_min=obs:0, cell_max=obs:1"};
  std::string grid_config{grid_bounds + "," + grid_cell_size + "," + cell_vars + "," + obs_min_max};
  m_grid = string2ConvexGrid(grid_config);
  if (m_grid.size() == 0)
    reportConfigWarning("Unable to generate grid for D* Lite algorithm due to bad config!");

  // post grid visuals
  m_grid.set_label("D* Lite");
  if ((m_post_visuals)  && (m_grid.size() != 0))
    Notify("VIEW_GRID", m_grid.get_spec());

  registerVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: registerVariables()

void DStarLite::registerVariables()
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

bool DStarLite::buildReport()
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
  m_msgs << "Tracked Obstacles: " << endl;
  for (auto const& obs : m_obstacle_map)
    m_msgs << obs.first << ";";
  m_msgs << endl;

  m_msgs << header << endl;
  m_msgs << "Path Stats:" << "todo" << endl;
  return(true);
}
