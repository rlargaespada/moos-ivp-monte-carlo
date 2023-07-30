/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IRIS2D.cpp                                        */
/*    DATE: June 14th, 2023                                 */
/************************************************************/

#include <cdd/setoper.h>
#include <cdd/cdd.h>
#include <cmath>
#include <iterator>
#include <set>
#include <string>
#include <vector>
#include "ACTable.h"
#include "GeomUtils.h"
#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsPoint.h"
#include "XYPolygon.h"
#include "XYPoint.h"
#include "iris_mosek.h"
#include "IRISPolygon.h"
#include "IRISProblem.h"
#include "IRIS2D.h"


//---------------------------------------------------------
// Constructor()

IRIS2D::IRIS2D()
{
  //* Config Variables
  // vars to subscribe to, all are set in onStartup();
  m_rebuild_iris_var = "";
  m_run_iris_var = "";
  m_clear_iris_var = "";
  m_obs_alert_var = "";
  m_seed_pt_var = "";

  // publication config
  m_iris_region_var = "IRIS_REGION";
  m_complete_var = "IRIS_COMPLETE";

  // visuals config
  m_label_color = "white";
  m_label_prefix = "";  // if empty, use GetAppName()

  m_post_poly_visuals = true;
  m_poly_fill_color = "violet";
  m_poly_edge_color = "blueviolet";
  m_poly_vert_color = "blueviolet";
  m_poly_edge_size = 1;
  m_poly_vert_size = 1;
  m_poly_transparency = 0.1;

  m_post_ellipse_visuals = true;
  m_ellipse_fill_color = "invisible";
  m_ellipse_edge_color = "turquoise";
  m_ellipse_vert_color = "turquoise";
  m_ellipse_edge_size = 1;
  m_ellipse_vert_size = 1;
  m_ellipse_transparency = 0.1;

  // IRIS config
  m_mode = "manual";  // "auto", "hybrid"
  m_desired_regions = 20;
  m_max_iters = IRIS_DEFAULT_MAX_ITERS;
  m_termination_threshold = IRIS_DEFAULT_TERMINATION_THRESHOLD;

  //* State Variables
  m_clear_pending = false;
  m_run_pending = false;
  m_active = false;

  m_iris_in_progress = false;
  m_iris_region_idx = -1;
  m_iris_start_time = 0;

  // set up cdd
  dd_set_global_constants();
}


//---------------------------------------------------------
// Destructor

IRIS2D::~IRIS2D()
{
  // cleanup cdd
  dd_free_global_constants();
}


//---------------------------------------------------------
// Procedure: OnNewMail()

bool IRIS2D::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key    = msg.GetKey();
    bool handled{true};

#if 0  // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    std::string sval  = msg.GetString();
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if (key == m_rebuild_iris_var) {
      m_run_pending = true;
      m_clear_pending = true;
    } else if (key == m_run_iris_var) {
      m_run_pending = true;
    } else if (key == m_clear_iris_var) {
      m_clear_pending = true;
    } else if (key == m_obs_alert_var) {
      handled = handleObstacleAlert(msg.GetString());
    } else if (key == "OBM_RESOLVED") {
      handleObstacleResolved(msg.GetString());  // even if returns false, mail is ok
    } else if (key == m_seed_pt_var) {
      XYPoint seed_pt{string2Point(msg.GetString())};
      if (seed_pt.valid())
        m_seed_pt_queue.push_back(seed_pt);
      else
        handled = false;
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      handled = false;
    }

    if (!handled)
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}


bool IRIS2D::handleObstacleAlert(std::string obs_alert)
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


bool IRIS2D::handleObstacleResolved(const std::string &obs_label)
{
  // if obstacle is in ADD queue for some reason, remove it
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

bool IRIS2D::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool IRIS2D::Iterate()
{
  AppCastingMOOSApp::Iterate();

  handleRequests();
  syncObstacles();

  // if there's an open IRIS problem, work on it
  if (m_iris_in_progress) {
    bool problem_complete{runIRIS()};

    if (problem_complete) {
      // if problem is complete, save and post final poly and ellipse
      bool region_valid{saveIRISRegion(m_iris_region_idx)};

      // check completion conditions
      if (region_valid && checkFinishConditions()) {
        Notify(m_complete_var, "now");
        if (m_mode == "manual")
          m_active = false;
      }
    }

  // otherwise set up an iris problem to run on the next iteration
  } else if (!m_seed_pt_queue.empty()) {
    // if we got a seed point in the mail, run IRIS around that point, even if inactive
    XYPoint seed{m_seed_pt_queue.front()};
    m_seed_pt_queue.pop_front();
    if (seedOK(seed)) {
      m_iris_region_idx = -1;  // add new safe region
      // todo (future): for seed point queue, want final region to contain the seed point
      setIRISProblem(seed);
      m_iris_in_progress = true;
    }

  } else if ((m_active) && (m_safe_regions.size() < m_desired_regions)) {
    // if we still have regions to build to get to the desired number, run IRIS
    // around a random seed point
    XYPoint seed{randomSeedPoint()};

    if (seed.valid()) {
      m_iris_region_idx = -1;  // add new safe region
      setIRISProblem(seed);
      m_iris_in_progress = true;
    }

  } else if ((m_active) && (!m_invalid_regions.empty())) {
    // if any regions are invalid because obstacles changed, replace a region
    // by running IRIS around a random point inside the region
    int region_idx{*m_invalid_regions.begin()};
    XYPolygon invalid_region{m_safe_regions.at(region_idx)};
    XYPoint seed{randomSeedPoint(invalid_region)};

    if (seed.valid()) {
      m_iris_region_idx = region_idx;  // replace invalid region with new one
      setIRISProblem(seed);
      m_iris_in_progress = true;
    }

  } else if ((m_active) && (m_mode == "manual")) {
    // nothing to do, go back to inactive
    Notify(m_complete_var, "now");
    m_active = false;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------

void IRIS2D::handleRequests()
{
  if (m_clear_pending) {
    // clear visuals
    if (m_post_poly_visuals) {
      for (XYPolygon region : m_safe_regions)
        Notify("VIEW_POLYGON", region.get_spec_inactive());
    }
    if (m_post_ellipse_visuals) {
      for (XYPolygon ellipse : m_iris_ellipses)
        Notify("VIEW_POLYGON", ellipse.get_spec_inactive());
    }

    // clear saved regions
    m_safe_regions.clear();
    m_iris_ellipses.clear();
    m_invalid_regions.clear();
    m_iris_in_progress = false;

    if (m_mode != "auto")
      m_active = false;
    reportEvent("All IRIS regions cleared!");
  }

  if (m_run_pending)
    m_active = true;  // always true in auto mode

  m_clear_pending = false;
  m_run_pending = false;
}


void IRIS2D::syncObstacles()
{
  // no obstacles to change, exit early
  if ((m_obstacle_add_queue.empty()) && (m_obstacle_remove_queue.empty()))
    return;

  // if any regions are no longer invalid (don't intersect with any obstacles),
  // remove them from the invalid set
  std::vector<int> no_longer_invalid;
  for (int region_idx : m_invalid_regions) {
    bool still_invalid{false};
    // only need to check obstacles being removed, not all obstacles
    for (std::string const& obs_label : m_obstacle_remove_queue) {
      XYPolygon region{m_safe_regions.at(region_idx)};
      XYPolygon obs{m_obstacle_map.at(obs_label)};

      if (region.intersects(obs)) {  // if any intersections, region is still invalid
        still_invalid = true;
        break;
      }
    }

    if (!still_invalid)
      no_longer_invalid.push_back(region_idx);
  }

  // update invalid set after iterating
  for (int region_idx : no_longer_invalid)
    m_invalid_regions.erase(region_idx);

  // if any safe regions intersect new obstacles, mark them as invalid
  for (int region_idx{0}; region_idx < m_safe_regions.size(); region_idx++) {
    for (auto const& obs : m_obstacle_add_queue) {
      if (m_safe_regions.at(region_idx).intersects(obs.second)) {
        m_invalid_regions.insert(region_idx);
        break;
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
}


// todo (future): more intelligently get new seed points
// see Deits paper on UAV path planning using IRIS for seed pt heuristc
XYPoint IRIS2D::randomSeedPoint(XYPolygon container)
{
  double x, y;

  // try to get a random point in bounds which doesn't intersect
  // any obstacles or existing iris regions
  for (int tries{0}; tries < 100; tries++) {
    randPointInPoly(container, x, y);
    bool point_ok{true};

    // try to pick a point outside existing valid iris regions
    // this check is more likely to fail since regions are bigger than
    // obstacles, so check this first
    if (tries < 95) {  // if we've tried a lot already, waive this check
      for (int i{0}; i < m_safe_regions.size(); i++) {
        if (m_invalid_regions.count(i))  // ignore invalid regions
          continue;

        if (m_safe_regions.at(i).contains(x, y)) {
          point_ok = false;
          break;
        }
      }
    }

    // check for intersections with obstacles
    if (point_ok) {  // don't bother with this check if the point is bad
      for (auto const &obs : m_obstacle_map) {
        if (obs.second.contains(x, y)) {
          point_ok = false;
          break;
        }
      }
    }

    // no intersections, return point
    if (point_ok)
      return (XYPoint(x, y));
  }

  return (XYPoint{});  // return invalid point if our attempts fail
}


bool IRIS2D::checkFinishConditions()
{
  // if we just added the last region to get to the desired number, post completion
  if ((m_iris_region_idx == -1) && (m_safe_regions.size() == m_desired_regions))
    return (true);

  // if we just replaced the last invalid region, post completion
  if ((m_iris_region_idx > -1) && (m_invalid_regions.empty()))
    return (true);

  return (false);
}


//---------------------------------------------------------

bool IRIS2D::seedOK(const XYPoint &seed)
{
  std::string msg1{"Cannot build region around x="};
  msg1 += doubleToStringX(seed.x()) + ", y=" + doubleToStringX(seed.y());
  std::string msg2;

  // make sure seed is valid
  if (!seed.valid()) {
    msg2 = "This seed point is marked invalid!";
  // make sure seed is inside bounds
  } else if (!m_iris_bounds.contains(seed)) {
    msg2 = "This point is outside the bounds of the IRIS search!";
  // make sure seed doesn't intersect any obstacles
  } else {
    for (auto const &obs : m_obstacle_map) {
      if (obs.second.contains(seed)) {
        msg2 = "This point is inside an obstacle!";
        break;
      }
    }
  }

  if (!msg2.empty()) {
    reportRunWarning(msg2);  // report msg2 first so msg1 appears on top visually
    reportRunWarning(msg1);
    return (false);
  }

  return (true);
}


void IRIS2D::setIRISProblem(const XYPoint &seed)
{
  if (!m_xy_iris_bounds.is_convex()) {
    reportRunWarning("Cannot run IRIS because bounding polygon has not been set properly!");
    m_active = false;
    return;
  }

  m_current_problem = IRISProblem(seed, m_iris_bounds, m_max_iters, m_termination_threshold);
  for (auto const &obs : m_obstacle_map)
    m_current_problem.addObstacle(obs.second);
  reportEvent("Initialized IRISProblem around seed point " + seed.get_spec());
  m_iris_in_progress = true;
}


bool IRIS2D::runIRIS()
{
  if (m_current_problem.complete())
    return (true);

  if (m_current_problem.itersDone() == 0)  // running a new problem, save start time
    m_iris_start_time = MOOSTime();

  // calc number of IRIS iterations to execute
  double time_alloted{1/(GetAppFreq() * m_time_warp)};  // approx. time in s allotted per app iter
  double num_iters{std::floor(time_alloted / .02)};  // IRIS iters usually take under 0.2s
  if (num_iters < 1)
    num_iters = 1;

  // run IRIS. if there's an exception, report run warning and scrap the current problem
  bool problem_complete{false};
  try {
    problem_complete = m_current_problem.run(static_cast<int>(num_iters));
    m_iris_in_progress = !problem_complete;
  } catch (const IRISMosekError& execption) {
    reportRunWarning("Recieved IRIS Mosek error!");
    reportRunWarning(execption.what());

    problem_complete = false;  // prevents caller from thinking optimization succeeded
    m_iris_in_progress = false;  // stops app from continuing to work on this problem
  } catch (const InnerEllipsoidInfeasibleError& execption) {
    reportRunWarning("Recieved IRIS inner ellipsoid infeasible error!");
    reportRunWarning(execption.what());

    problem_complete = false;  // prevents caller from thinking optimization succeeded
    m_iris_in_progress = false;  // stops app from continuing to work on this problem
  }

  return problem_complete;
}


bool IRIS2D::saveIRISRegion(int idx)
{
  // parse polygon from IRISProblem and save IRIS stats
  XYPolygon poly{m_current_problem.getPolygon()};
  IRISStats stats{
    idx == -1 ? m_safe_regions.size() : idx,
    MOOSTime() - m_iris_start_time,  // duration
    m_current_problem.itersDone(),  // num_iters
    poly.is_convex()  // valid
  };

  m_iris_stats.push_front(stats);
  while (m_iris_stats.size() > 8)  // limit stats queue to 8 members
    m_iris_stats.pop_back();

  // if polygon is invalid, return without saving
  if (!poly.is_convex())
    return (false);

  // set visual params for polygon
  poly.set_color("edge", m_poly_edge_color);
  poly.set_color("vertex", m_poly_vert_color);
  poly.set_color("fill", m_poly_fill_color);
  poly.set_color("label", m_label_color);
  poly.set_vertex_size(m_poly_vert_size);
  poly.set_edge_size(m_poly_edge_size);
  poly.set_transparency(m_poly_transparency);

  // save poly with correct label
  if (idx == -1) {
    poly.set_label(m_label_prefix + "_polygon_" + intToString(m_safe_regions.size()));
    m_safe_regions.push_back(poly);
  } else {
    m_invalid_regions.erase(idx);
    poly.set_label(m_label_prefix + "_polygon_" + intToString(idx));
    m_safe_regions.at(idx) = poly;
  }

  // post polygon and visuals
  Notify(m_iris_region_var, poly.get_spec_pts_label(3));
  if (m_post_poly_visuals)
    Notify("VIEW_POLYGON", poly.get_spec(3));


  // create ellipse and set visual params
  XYPolygon ellipse{m_current_problem.getEllipse()};
  ellipse.set_color("edge", m_ellipse_edge_color);
  ellipse.set_color("vertex", m_ellipse_vert_color);
  ellipse.set_color("fill", m_ellipse_fill_color);
  ellipse.set_color("label", "invisible");
  ellipse.set_vertex_size(m_ellipse_vert_size);
  ellipse.set_edge_size(m_ellipse_edge_size);
  ellipse.set_transparency(m_ellipse_transparency);

  // save ellipse with correct label
  if (idx == -1) {
    ellipse.set_label(m_label_prefix + "_ellipse_" + intToString(m_iris_ellipses.size()));
    m_iris_ellipses.push_back(ellipse);
  } else {
    ellipse.set_label(m_label_prefix + "_ellipse_" + intToString(idx));
    m_iris_ellipses.at(idx) = ellipse;
  }

  // post ellipse visuals
  if (m_post_ellipse_visuals)
    Notify("VIEW_POLYGON", ellipse.get_spec(3));
  return (true);
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool IRIS2D::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  std::string iris_bounds{"pts="};

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    // subscription config
    if (param == "iris_rebuild_var") {
      handled = setNonWhiteVarOnString(m_rebuild_iris_var, toupper(value));
    } else if (param == "iris_run_var") {
      handled = setNonWhiteVarOnString(m_run_iris_var, toupper(value));
    } else if (param == "iris_clear_var") {
      handled = setNonWhiteVarOnString(m_clear_iris_var, toupper(value));
    } else if (param == "obs_alert_var") {
      handled = setNonWhiteVarOnString(m_obs_alert_var, toupper(value));
    } else if ((param == "seed_point_var") || (param == "seed_pt_var")) {
      handled = setNonWhiteVarOnString(m_seed_pt_var, toupper(value));
    // publication config
    } else if (param == "iris_region_var") {
      handled = setNonWhiteVarOnString(m_iris_region_var, toupper(value));
    } else if (param == "iris_complete_var") {
      handled = setNonWhiteVarOnString(m_complete_var, toupper(value));
    } else if (param == "label_prefix") {
      handled = setNonWhiteVarOnString(m_label_prefix, value);
    // visual publication config
    } else if ((param == "label_color") && isColor(value)) {
      handled = setColorOnString(m_label_color, value);

    } else if (param == "post_polygons") {
      handled = setBooleanOnString(m_post_poly_visuals, value);
    } else if ((param == "poly_fill_color") && isColor(value)) {
      handled = setColorOnString(m_poly_fill_color, value);
    } else if ((param == "poly_edge_color") && isColor(value)) {
      handled = setColorOnString(m_poly_edge_color, value);
    } else if ((param == "poly_vert_color") && isColor(value)) {
      handled = setColorOnString(m_poly_vert_color, value);
    } else if (param == "poly_edge_size") {
      handled = setNonNegDoubleOnString(m_poly_edge_size, value);
    } else if (param == "poly_vert_size") {
      handled = setNonNegDoubleOnString(m_poly_vert_size, value);
    } else if (param == "poly_transparency") {
      handled = setNonNegDoubleOnString(m_poly_transparency, value);

    } else if (param == "post_ellipses") {
      handled = setBooleanOnString(m_post_ellipse_visuals, value);
    } else if ((param == "ellipse_fill_color") && isColor(value)) {
      handled = setColorOnString(m_ellipse_fill_color, value);
    } else if ((param == "ellipse_edge_color") && isColor(value)) {
      handled = setColorOnString(m_ellipse_edge_color, value);
    } else if ((param == "ellipse_vert_color") && isColor(value)) {
      handled = setColorOnString(m_ellipse_vert_color, value);
    } else if (param == "ellipse_edge_size") {
      handled = setNonNegDoubleOnString(m_ellipse_edge_size, value);
    } else if (param == "ellipse_vert_size") {
      handled = setNonNegDoubleOnString(m_ellipse_vert_size, value);
    } else if (param == "ellipse_transparency") {
      handled = setNonNegDoubleOnString(m_ellipse_transparency, value);

    // IRIS config
    } else if (param == "mode") {
      std::set<std::string> valid_modes{"manual", "auto", "hybrid"};
      if (valid_modes.count(tolower(value)) == 0) {
        reportUnhandledConfigWarning(
          "Mode must be one of " + stringSetToString(valid_modes, ',') +
          ". Defaulting to " + m_mode + "mode.");
        handled = false;
      } else {
        m_mode = tolower(value);
        handled = true;
      }
    } else if (param == "desired_regions") {
      handled = setPosUIntOnString(m_desired_regions, value);
    } else if (param == "iris_bounds") {
      if (!strBegins(value, "{")) value = "{" + value;
      if (!strEnds(value, "}")) value += "}";
      iris_bounds += value;
      handled = true;
    } else if (param == "max_iters") {
      handled = setPosUIntOnString(m_max_iters, value);
    } else if (param == "termination_threshold") {
      handled = setPosDoubleOnString(m_termination_threshold, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // set subscription vars if unset
  if (m_rebuild_iris_var.empty())
    m_rebuild_iris_var = "REBUILD_IRIS";
  if (m_run_iris_var.empty())
    m_run_iris_var = "RUN_IRIS";
  if (m_clear_iris_var.empty())
    m_clear_iris_var = "CLEAR_IRIS";
  if (m_obs_alert_var.empty())
    m_obs_alert_var = "OBSTACLE_ALERT";
  if (m_seed_pt_var.empty())
    m_seed_pt_var = "IRIS_SEED_POINT";

  // additional setup
  if (m_label_prefix.empty())  // if no prefix is given use name of app
    m_label_prefix = GetAppName();
  if (m_mode == "auto")
    m_active = true;  // in auto mode app is always active

  // save IRIS bounds and post visuals if needed
  m_xy_iris_bounds = string2Poly(iris_bounds);
  if (!m_xy_iris_bounds.is_convex()) {
    reportConfigWarning("Invalid IRIS bounds provided");
    m_active = false;
  }
  m_iris_bounds = IRISPolygon(m_xy_iris_bounds);

  if (m_post_poly_visuals) {
    m_xy_iris_bounds.set_label(m_label_prefix + "_bounds");
    m_xy_iris_bounds.set_color("edge", m_poly_edge_color);
    m_xy_iris_bounds.set_color("vertex", m_poly_vert_color);
    m_xy_iris_bounds.set_color("fill", "invisible");
    m_xy_iris_bounds.set_color("label", m_label_color);
    m_xy_iris_bounds.set_vertex_size(m_poly_vert_size * 2);  // make lines a bit thicker
    m_xy_iris_bounds.set_edge_size(m_poly_edge_size * 2);  // make lines a bit thicker
    m_xy_iris_bounds.set_transparency(1);
    Notify("VIEW_POLYGON", m_xy_iris_bounds.get_spec());
  }

  registerVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: registerVariables()

void IRIS2D::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  if (!m_rebuild_iris_var.empty())
    Register(m_rebuild_iris_var, 0);
  if (!m_run_iris_var.empty())
    Register(m_run_iris_var, 0);
  if (!m_clear_iris_var.empty())
    Register(m_clear_iris_var, 0);
  Register("OBM_RESOLVED", 0);
  if (!m_obs_alert_var.empty())
    Register(m_obs_alert_var, 0);
  if (!m_seed_pt_var.empty())
    Register(m_seed_pt_var, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool IRIS2D::buildReport()
{
  using std::endl;
  std::string header{"================================"};

  m_msgs << header << endl;
  m_msgs << "Subscriptions:" << endl;
  m_msgs << "  iris_rebuild_var: " << m_rebuild_iris_var << endl;
  m_msgs << "  iris_run_var: " << m_run_iris_var << endl;
  m_msgs << "  iris_clear_var: " << m_clear_iris_var << endl;
  m_msgs << "  obs_alert_var: " << m_obs_alert_var << endl;
  m_msgs << "  seed_point_var: " << m_seed_pt_var << endl;
  m_msgs << "Publications: " << endl;
  m_msgs << "  iris_region_var: " << m_iris_region_var << endl;
  m_msgs << "  iris_complete_var: " << m_complete_var << endl;
  m_msgs << "  label_prefix: " << m_label_prefix << endl;
  m_msgs << "  post_polygons: " << boolToString(m_post_poly_visuals) << endl;
  m_msgs << "  post_ellipses: " << boolToString(m_post_ellipse_visuals) << endl;

  m_msgs << header << endl;
  m_msgs << "IRIS Config:" << endl;
  m_msgs << "  mode: " << m_mode << endl;
  m_msgs << "  desired_regions: " << uintToString(m_desired_regions) << endl;
  m_msgs << "  iris_bounds: " << m_xy_iris_bounds.get_spec_pts(2) << endl;
  m_msgs << "  max_iters: " << uintToString(m_max_iters) << endl;
  m_msgs << "  termination_threshold: " << doubleToStringX(m_termination_threshold) << endl;

  m_msgs << header << endl;
  m_msgs << "IRIS State:" << endl;
  m_msgs << "  Active: " << boolToString(m_active) << endl;
  m_msgs << "  Regions Found: " << uintToString(m_safe_regions.size()) << endl;
  m_msgs << "  Obstacles Tracked: " << uintToString(m_obstacle_map.size()) << endl;
  m_msgs << "  IRIS in Progress: " << boolToString(m_iris_in_progress) << endl;
  m_msgs << "  Invalid Regions: " << uintToString(m_invalid_regions.size()) << endl;

  m_msgs << header << endl;
  m_msgs << "Most Recent IRIS Stats" << endl;
  ACTable actab(4);
  actab << "Region # | Build Time | Iterations | Valid";
  actab.addHeaderLines();
  for (const IRISStats & s : m_iris_stats) {
    actab << uintToString(s.idx) << doubleToStringX(s.duration, 3) + "s" <<
      intToString(s.num_iters) << boolToString(s.valid);
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
