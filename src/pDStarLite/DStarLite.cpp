/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DStarLite.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "ACTable.h"
#include "DStarLite.h"
#include "GeomUtils.h"
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
#include "XYSegList.h"
#include "XYSquare.h"


// todo: optimize with more consts and references, use emplaces
// todo: create a base class for planning apps
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
  m_post_visuals = true;

  m_path_found_var = "PATH_FOUND";
  m_path_complete_var = "PATH_COMPLETE";
  m_path_stats_var = "PATH_STATS";
  m_path_failed_var = "PATH_FAILED";

  // D* Lite config
  m_max_iters = 200;

  //* State Variables
  m_start_cell = -1;  // set these to -1 to signify they're invalid
  m_goal_cell = -1;

  // planning state data
  m_mode = PlannerMode::IDLE;
  m_planning_start_time = 0;
  m_planning_end_time = 0;
  m_path_len_traversed = 0;

  // D* Lite state
  m_last_cell = -1;  // -1 is invalid
  m_k_m = 0;
  m_next_path_idx = 0;
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
        handleNewWpt(m_path.size() - 1);
      }
    } else if ((key == "NODE_REPORT_LOCAL") || (key == "NODE_REPORT")) {
      std::string report{tolower(msg.GetString())};
      double xval{tokDoubleParse(report, "x", ',', '=')};
      double yval{tokDoubleParse(report, "y", ',', '=')};
      m_vpos.set_vertex(xval, yval);
    } else if (key == "WPT_INDEX") {
      // add just finished segment of path to dist traveled
      if (m_mode == PlannerMode::IN_TRANSIT)
        handleNewWpt(static_cast<int>(msg.GetDouble()));
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
  // add it to the REMOVE queue so we can remove the old
  // version of the obstacle in the iterate loop
  if (m_obstacle_map.count(key))
    m_obstacle_remove_queue.insert(key);

  return (true);
}


bool DStarLite::handleObstacleResolved(const std::string obs_label)
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


void DStarLite::handleNewWpt(int new_wpt_idx)
{
  int wpt_just_completed{new_wpt_idx - 1};
  int prev_wpt_completed{std::max(m_next_path_idx - 1, 0)};

  if (wpt_just_completed < 1)
    return;

  for (int i = prev_wpt_completed; i < wpt_just_completed; i++) {
    double x0{m_path.get_vx(i)}, y0{m_path.get_vy(i)};
    double x1{m_path.get_vx(i + 1)}, y1{m_path.get_vy(i + 1)};
    m_path_len_traversed += distPointToPoint(x0, y0, x1, y1);
  }

  m_next_path_idx = new_wpt_idx;
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
    m_path_grid_cells.clear();

    // plan path until we reach max number of iterations
    path_found = planPath();

  // if already planning a path, pick up from where we left off
  } else if (m_mode == PlannerMode::PLANNING_IN_PROGRESS) {
    path_found = planPath();
  // if transiting, check if we need to replan and replan if needed
  } else if (m_mode == PlannerMode::IN_TRANSIT) {
    if (!checkObstacles()) {
      Notify(m_prefix + m_path_found_var, "false");
      postFlags(m_replan_flags);

      // plan path until we reach max number of iterations
      m_planning_start_time = MOOSTime();
      path_found = planPath();
    }
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

    // only check REMOVE queues if cell has an obstacle
    if (m_grid.getVal(ix, cix)) {
      // if grid cell intersects with an obstacle that is marked to remove,
      // mark the cell to be cleared
      for (auto const& obs : m_obstacle_remove_queue) {
        if (m_obstacle_map[obs].intersects(grid_cell)) {
          cell_clear_pending = true;
          break;  // only need to check for a single intersection
        }
      }
    }

    // if cell is marked to be cleared, check if it intersects with any
    // obstacle that should NOT be removed. if so, don't clear the cell
    if (cell_clear_pending) {
      for (auto const& obs : m_obstacle_map) {
        // if obstacle is on REMOVE queue, we don't need to check again
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

    // only check ADD queue if cell doesn't have an obstacle
    if (!m_grid.getVal(ix, cix)) {
      // if grid cell intersects with an obstacle to add, set cell to 1
      for (auto const& obs : m_obstacle_add_queue) {
        if (obs.second.intersects(grid_cell)) {
          m_grid.setVal(ix, 1, cix);
          update.addUpdate(ix, "obs", 1);
          break;  // only need to check for a single intersection
        }
      }
    }
  }

  // update obstacle map and clear queues
  for (auto const& obs : m_obstacle_remove_queue)
    m_obstacle_map.erase(obs);
  for (auto const& obs : m_obstacle_add_queue)
    m_obstacle_map[obs.first] = obs.second;
  m_obstacle_add_queue.clear();
  m_obstacle_remove_queue.clear();

  if (update.size() == 0)
    return;

  // post visuals
  if (m_post_visuals) Notify("VIEW_GRID_DELTA", update.get_spec());

  // update vertices when in transit
  if (m_mode == PlannerMode::IN_TRANSIT) {
    for (int i = 0; i < update.size(); i++)
      updateVertex(update.getCellIX(i));
  }
}


bool DStarLite::planPath()
{
  if (!checkPlanningPreconditions()) {
    handlePlanningFail();  // checkPlanningPreconditions posts its own warnings
    return (false);
  }

  bool planning_complete{false};

  //* D* Lite Algorithm
  // got a request for a new path, start from scratch
  if (m_mode == PlannerMode::REQUEST_PENDING) {
    // figure out where in the grid the start and goal points are
    m_start_cell = findCellByPoint(m_start_point);
    m_goal_cell = findCellByPoint(m_goal_point);
    if ((m_start_cell < 0) || (m_goal_cell < 0)) {
      handlePlanningFail("ERROR: tried to plan a path between points, where at least "
                         "one point was outside the search grid!");
      return (false);
    }

    // initialize and compute path
    m_last_cell = m_start_cell;
    initializeDStarLite();
    // end after D* Lite initialization, start planning during next Iterate() call

  // if we were already planning, pick up from where we left off
  } else if (m_mode == PlannerMode::PLANNING_IN_PROGRESS) {
    planning_complete = computeShortestPath(m_max_iters);

  // if we're in transit, plan again from the current vehicle position
  } else if (m_mode == PlannerMode::IN_TRANSIT) {
    m_start_point = m_vpos;  // start point is current vehicle position
    m_start_cell = findCellByPoint(m_vpos);
    if (m_start_cell < 0) {
      handlePlanningFail("ERROR: tried to replan from current vehicle position, "
                         "but vehicle has drifted outside the search grid!");
      return (false);
    }

    // if vehicle position has an obstacle, which can happen when skirting a newly
    // detected obstacle, clear the current cell
    unsigned int obs_cix{m_grid.getCellVarIX("obs")};
    m_grid.setVal(m_start_cell, 0, obs_cix);

    m_k_m += heuristic(m_last_cell, m_start_cell);
    // end after setting up for replanning, start planning during next Iterate() call
  }

  // planning done, return true if we found a path
  if (planning_complete) {
    bool path_ok{parsePathFromGrid()};
    if (!path_ok) {
      handlePlanningFail("Unable to find a path from the start to the goal!");
      return (false);
    }
    return (true);  // if planning worked and we got a path
  }

  // planning not done yet, return false
  m_mode = PlannerMode::PLANNING_IN_PROGRESS;
  return (false);
}


void DStarLite::handlePlanningFail(std::string warning_msg)
{
  if (!warning_msg.empty()) reportRunWarning(warning_msg);
  reportRunWarning("Planning failed!");
  m_mode = PlannerMode::PLANNING_FAILED;
  Notify(m_prefix + m_path_failed_var, "true");
}


//---------------------------------------------------------

int DStarLite::findCellByPoint(XYPoint pt)
{
  for (int ix = 0; ix < m_grid.size(); ix++) {
    if (m_grid.getElement(ix).containsPoint(pt.x(), pt.y()))
      return (ix);
  }
  return (-1);  // -1 if point not in graph
}


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
    if (m_grid.ptIntersect(neighbor_ix, neighbor_cx, neighbor_cy)) {
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
    if (m_grid.ptIntersect(neighbor_ix, neighbor_cx, neighbor_cy)) {
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


int DStarLite::getNextCell()
{
  int min_cell{-1};
  dsl_key min_key{INFINITY, INFINITY};
  for (auto const& cell : m_dstar_queue) {
    if (cell.second < min_key) {
      min_cell = cell.first;
      min_key = cell.second;
    }
  }
  return (min_cell);
}


// Euclidean distance between cells
double DStarLite::heuristic(int cell1, int cell2)
{
  double x1{m_grid.getElement(cell1).getCenterX()};
  double y1{m_grid.getElement(cell1).getCenterY()};
  double x2{m_grid.getElement(cell2).getCenterX()};
  double y2{m_grid.getElement(cell2).getCenterY()};
  return (distPointToPoint(x1, y1, x2, y2));
}


// Infinite if either cell is an obstacle or cells are not neighbors,
// else Euclidean dist * obstacle_scaling
double DStarLite::cost(int cell1, int cell2)
{
  unsigned int obs_cix{m_grid.getCellVarIX("obs")};
  if (m_grid.getVal(cell1, obs_cix) == 1) return (INFINITY);
  if (m_grid.getVal(cell2, obs_cix) == 1) return (INFINITY);
  if (!getNeighbors(cell1).count(cell2)) return (INFINITY);

  double base_cost{heuristic(cell1, cell2)};  // heuristic is Euclidean dist

  // if cell2 neighbors any obstacles, scale the cost
  int cost_scale{1};
  for (int n : getNeighbors(cell2)) {
    if (m_grid.getVal(n, obs_cix) == 1)
      cost_scale++;
  }

  // unused alternate cost scaling based on reciprocal distance to obstacles
  // slower, paths aren't as good; keeping here for future reference
  // double x2{m_grid.getElement(cell2).getCenterX()};
  // double y2{m_grid.getElement(cell2).getCenterY()};
  // double cost_scale{1};
  // for (auto const& obs : m_obstacle_map)
  //   cost_scale += (1. / obs.second.dist_to_poly(x2, y2));

  return (cost_scale * base_cost);
}


//---------------------------------------------------------

dsl_key DStarLite::calculateKey(int grid_ix)
{
  unsigned int g_cix{m_grid.getCellVarIX("g")}, rhs_cix{m_grid.getCellVarIX("rhs")};
  double g{m_grid.getVal(grid_ix, g_cix)}, rhs{m_grid.getVal(grid_ix, rhs_cix)};
  double first{std::min(g, rhs) + heuristic(m_start_cell, grid_ix) + m_k_m};
  double second{std::min(g, rhs)};
  return (dsl_key{first, second});
}


void DStarLite::initializeDStarLite()
{
  // clear priority queue, k_m
  m_dstar_queue.clear();
  m_k_m = 0;

  // initialize g and rhs values
  unsigned int g_cix{m_grid.getCellVarIX("g")}, rhs_cix{m_grid.getCellVarIX("rhs")};
  for (int ix = 0; ix < m_grid.size(); ix++) {
    m_grid.setVal(ix, INFINITY, g_cix);
    m_grid.setVal(ix, INFINITY, rhs_cix);
  }

  // add goal state to priority queue
  m_grid.setVal(m_goal_cell, 0, rhs_cix);
  m_dstar_queue[m_goal_cell] = calculateKey(m_goal_cell);
}


void DStarLite::updateVertex(int grid_ix)
{
  unsigned int g_cix{m_grid.getCellVarIX("g")}, rhs_cix{m_grid.getCellVarIX("rhs")};
  unsigned int obs_cix{m_grid.getCellVarIX("obs")};

  if (grid_ix != m_goal_cell) {
    double rhs{INFINITY};
    for (const int& n : getNeighbors(grid_ix))
      rhs = std::min(rhs, cost(grid_ix, n) + m_grid.getVal(n, g_cix));
    m_grid.setVal(grid_ix, rhs, rhs_cix);
  }

  if (m_dstar_queue.count(grid_ix))
    m_dstar_queue.erase(grid_ix);

  double g{m_grid.getVal(grid_ix, g_cix)}, rhs{m_grid.getVal(grid_ix, rhs_cix)};
  if (g != rhs)
    m_dstar_queue[grid_ix] = calculateKey(grid_ix);
}


bool DStarLite::computeShortestPath(int max_iters)
{
  // initialize loop vars
  max_iters /= m_time_warp;  // preserve performance at higher time warps
  unsigned int g_cix{m_grid.getCellVarIX("g")}, rhs_cix{m_grid.getCellVarIX("rhs")};
  int iters{0};

  int u{getNextCell()};
  dsl_key k_old{m_dstar_queue[u]};
  double g_u, rhs_u;

  while ((k_old < calculateKey(m_start_cell)) ||
         (m_grid.getVal(m_start_cell, rhs_cix) != m_grid.getVal(m_start_cell, g_cix))) {
    m_dstar_queue.erase(u);  // "pop" u from queue

    // get data for current node
    dsl_key key_u{calculateKey(u)};
    g_u = m_grid.getVal(u, g_cix);
    rhs_u = m_grid.getVal(u, rhs_cix);

    // update queue or update vertices as needed
    if (k_old < key_u) {
      m_dstar_queue[u] = key_u;
    } else if (g_u > rhs_u) {
      m_grid.setVal(u, rhs_u, g_cix);
      for (const int& n : getNeighbors(u))
        updateVertex(n);
    } else {
      m_grid.setVal(u, INFINITY, g_cix);
      for (const int& n : getNeighbors(u))
        updateVertex(n);
      updateVertex(u);
    }

    // if out of time, exit
    iters += 1;
    if (iters > m_max_iters)
      return (false);  // didn't find a path within allotted iterations

    // if queue is empty, planning has failed; return true so other functions handle it
    if (m_dstar_queue.empty())
      return (true);

    // prep for next loop
    u = getNextCell();
    k_old = m_dstar_queue[u];
  }

  return (true);
}


bool DStarLite::parsePathFromGrid()
{
  XYSegList path;
  std::vector<int> path_grid_cells;
  path.set_label("D* Lite");
  unsigned int g_cix{m_grid.getCellVarIX("g")};

  int current_cell{m_start_cell}, next_cell;
  double wpt_x, wpt_y, min_cost, neighbor_cost;
  double start{MOOSTime()};
  while (current_cell != m_goal_cell) {
    // check no path found condition, don't set path if not found
    if (std::isinf(m_grid.getVal(current_cell, g_cix))) {
      return (false);
    }

    // add current cell to path
    path_grid_cells.push_back(current_cell);
    wpt_x = m_grid.getElement(current_cell).getCenterX();
    wpt_y = m_grid.getElement(current_cell).getCenterY();
    path.add_vertex(wpt_x, wpt_y);

    // determine next cell on the path
    next_cell = -1;
    min_cost = INFINITY;
    for (const int& n : getNeighbors(current_cell)) {
      neighbor_cost = cost(current_cell, n) + m_grid.getVal(n, g_cix);
      if (neighbor_cost < min_cost) {
        next_cell = n;
        min_cost = neighbor_cost;
      }
    }
    current_cell = next_cell;

    if ((MOOSTime() - start) > 10)  // if we somehow get stuck here, kill after 10 seconds
      return (false);
  }

  path.mod_vertex(0, m_start_point.x(), m_start_point.y());  // change first point to be start point
  path_grid_cells.push_back(m_goal_cell);
  path.add_vertex(m_goal_point);  // add goal point

  // save path and return
  m_path = path;
  m_path_grid_cells = path_grid_cells;
  return (true);
}


//---------------------------------------------------------

std::string DStarLite::getPathStats()
{
  std::vector<std::string> stats;

  std::string s{"algorithm=D* Lite"};
  stats.push_back(s);
  s = "planning_time=" + doubleToStringX(m_planning_end_time - m_planning_start_time, 2);
  stats.push_back(s);
  s = "path_len_to_go=" + doubleToStringX(m_path.length(), 2);
  stats.push_back(s);
  s = "path_len_traversed=" + doubleToStringX(m_path_len_traversed, 2);
  stats.push_back(s);

  return stringVectorToString(stats, ',');
}


bool DStarLite::postPath()
{
  Notify(m_prefix + m_path_found_var, "true");
  Notify(m_prefix + m_path_stats_var, getPathStats());
  m_next_path_idx = 0;
  postFlags(m_traverse_flags);
  return (true);
}


//---------------------------------------------------------

bool DStarLite::checkObstacles()
{
  // if we strayed too far from the next waypoint, replan is needed
  XYPoint next_wpt{m_path.get_point(m_next_path_idx)};
  double threshold{3 * m_grid.getCellSize()};
  if (distPointToPoint(m_vpos, next_wpt) > threshold)
    return (false);

  // check each waypoint on remainder of path for validity
  unsigned int obs_cix{m_grid.getCellVarIX("obs")};
  for (int i = m_next_path_idx; i < m_path.size(); i++) {
    // if this waypoint is in an occupied cell, need to replan
    int cell1{m_path_grid_cells[i]};
    if (m_grid.getVal(cell1, obs_cix) == 1)
      return (false);

    // if we're not at the end of the path, check if the line between
    // this waypoint and the next intersects any obstacle
    //* NOTE: this case is covered by previous check on occupied squares and thus is unused
    // if (i < m_path.size() - 1) {
    //   // get xy values of this point and the next point
    //   double x1{m_path.get_vx(i)}, y1{m_path.get_vy(i)};
    //   double x2{m_path.get_vx(i + 1)}, y2{m_path.get_vy(i + 1)};

    //   // if line between points intersects an obstacle, replan is needed
    //   for (auto const& obs : m_obstacle_map) {
    //     if (obs.second.seg_intercepts(x1, y1, x2, y2))
    //       return (false);
    //   }
    // }
  }
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
    } else if ((param == "init_plan_flag") || param == "initflag") {
      handled = addVarDataPairOnString(m_init_plan_flags, value);
    } else if ((param == "traverse_flag") || param == "traverseflag") {
      handled = addVarDataPairOnString(m_traverse_flags, value);
    } else if ((param == "replan_flag") || param == "replanflag") {
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
  if (m_path_request_var.empty()) m_path_request_var = "PLAN_PATH_REQUESTED";
  if (m_obs_alert_var.empty()) m_obs_alert_var = "OBSTACLE_ALERT";
  if (m_wpt_complete_var.empty()) m_wpt_complete_var = "WAYPOINTS_COMPLETE";

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
  Register("WPT_INDEX", 0);
  if (!m_path_request_var.empty()) Register(m_path_request_var, 0);
  if (!m_obs_alert_var.empty()) Register(m_obs_alert_var, 0);
  if (!m_wpt_complete_var.empty()) Register(m_wpt_complete_var, 0);
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
  if (!m_prefix.empty())
    m_msgs << "Publication Prefix: " << m_prefix << endl;

  m_msgs << header << endl;
  m_msgs << "Tracked Obstacles: " << endl;
  for (auto const& obs : m_obstacle_map)
    m_msgs << obs.first << ";";
  m_msgs << endl;

  m_msgs << header << endl;
  m_msgs << "D* Lite State:" << endl;
  m_msgs << "  Start Cell: " << intToString(m_start_cell)  << endl;
  m_msgs << "  Goal Cell: " << intToString(m_goal_cell)  << endl;
  m_msgs << "  Cells in Queue: "  << intToString(m_dstar_queue.size())  << endl;

  m_msgs << header << endl;
  if (m_mode == PlannerMode::PLANNING_IN_PROGRESS) {
    m_msgs << "PLANNING IN PROGRESS" << endl;
    return (true);
  }

  m_msgs << "Path Stats:" << endl;
  m_msgs << "  Next Waypoint Index: " << intToString(m_next_path_idx) << endl;
  m_msgs << "  Planning Time: " <<
    doubleToStringX(m_planning_end_time - m_planning_start_time, 2) << endl;
  m_msgs << "  Path Length: " << doubleToStringX(m_path.length(), 2) << endl;
  m_msgs << "  Path Length Previously Traversed: " <<
    doubleToStringX(m_path_len_traversed, 2) << endl;
  return(true);
}
