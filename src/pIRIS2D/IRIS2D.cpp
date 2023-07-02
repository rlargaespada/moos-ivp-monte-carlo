/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IRIS2D.cpp                                        */
/*    DATE: June 14th, 2023                                 */
/************************************************************/

#include <cdd/setoper.h>
#include <cdd/cdd.h>
#include <Eigen/Dense>
#include <mosek.h>
#include <cmath>
#include <iterator>
#include <set>
#include <string>
#include <vector>
#include "ACTable.h"
#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsPoint.h"
#include "XYPolygon.h"
#include "XYPoint.h"
#include "IRISPolygon.h"
#include "IRISProblem.h"
#include "IRIS2D.h"


//---------------------------------------------------------
// Constructor()

IRIS2D::IRIS2D()
{
  //* Config Variables
  // vars to subscribe to, all are set in onStartup();
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
  m_poly_transparency = 0.15;

  m_post_ellipse_visuals = true;
  m_ellipse_fill_color = "invisible";
  m_ellipse_edge_color = "turquoise";
  m_ellipse_vert_color = "turquoise";
  m_ellipse_edge_size = 1;
  m_ellipse_vert_size = 1;
  m_ellipse_transparency = 0.15;

  m_ellipse_edge_size = 1;
  m_ellipse_transparency = 0.15;

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

    if (key == "RUN_IRIS") {
      m_run_pending = true;
    } else if (key == "CLEAR_IRIS") {
      m_run_pending = false;
      m_clear_pending = true;
    } else if (key == m_obs_alert_var) {
      handled = handleObstacleAlert(msg.GetString());
    } else if (key == "OBM_RESOLVED") {
      handleObstacleResolved(msg.GetString());  // even if returns false, mail is ok
    } else if (key == m_seed_pt_var) {
      XYPoint seed_pt{string2Point(msg.GetString())};
      if (seed_pt.valid())
        m_seed_pt_queue.push(seed_pt);
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

  /*
  if currently in the middle of an iris problem
    continue running problem
  else if seed point queue isn't empty
    buildRegion(seed point)
  else if active and still regions to go
    buildRegion(random seed)
    if manual and last region
      mark inactive, post complete
  else if active and invalidate queue isn't empty
    erase region in invalidate queue
    buildRegion(center of removed region)
    if queue is empty
      post complete
      if manual
        mark inactive
  else if active and manual
    go inactive

  buildRegion(seed point)
    set up problem
      polygon starts out empty
      ellipse starts out from seed point
      bounds from pIRIS2D
      obstacles from newest obstacle map
      input: seed XYPoint, obstacles map (?), bounds matrix/IRISPoly
    while below iter limit
      separating_hyperplanes(problem.obstacles, problem.ellipsoid, new_poly)
      add bounds as constraints to new_poly
      save new poly to problem
      volume = iris::mosek inner_ellipsoid(polygon, ellipsoid)
      save new volume
      check termination conditions
    if done
      save polygon as XYPolygon
      post VIEW_OVAL, VIEW_POLYGON
    otherwise pick it up next time


  first pass at IRIS: when posting to seed point, build region
  */

  handleRequests();
  syncObstacles();

  // todo: handle splitting IRIS into multiple iterations
  // if (m_iris_in_progress) {
  // } else if (!m_seed_pt_queue.empty()) {
  if (!m_seed_pt_queue.empty()) {
    setIRISProblem(m_seed_pt_queue.front());
    m_seed_pt_queue.pop();

    runIRIS();
    saveIRISRegion();

    // bool iris_complete{runIRIS()};
    // if (iris_complete) {
    //   saveIRISRegion();
    //   m_iris_in_progress = false;
    // } else {
    //   m_iris_in_progress = true;
    // }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------

void IRIS2D::handleRequests()
{
  if (m_clear_pending) {
    // todo: clear visuals
    m_safe_regions.clear();
    m_iris_ellipses.clear();
    if (m_mode != "auto")
      m_active = false;
  } else if (m_run_pending) {
    m_active = true;  // always true in auto mode
  }

  m_clear_pending = false;
  m_run_pending = false;
}


void IRIS2D::syncObstacles()
{
  // no obstacles to change, exit early
  if ((m_obstacle_add_queue.empty()) && (m_obstacle_remove_queue.empty()))
    return;

  //! disable invalid region checks while testing
  // // if any obstacles are no longer invalid (don't intersect with any obstacles),
  // // remove them from the invalid set
  // std::vector<int> no_longer_invalid;
  // for (int region_idx : m_invalid_regions) {
  //   bool still_invalid{false};
  //   // only need to check obstacles being removed, not all obstacles
  //   for (std::string obs_label : m_obstacle_remove_queue) {
  //     XYPolygon region{m_safe_regions.at(region_idx)};
  //     XYPolygon obs{m_obstacle_map.at(obs_label)};

  //     if (region.intersects(obs)) {  // if any intersections, region is still invalid
  //       still_invalid = true;
  //       break;
  //     }
  //   }

  //   if (!still_invalid)
  //     no_longer_invalid.push_back(region_idx);
  // }

  // // update invalid set after iterating
  // for (int region_idx : no_longer_invalid)
  //   m_invalid_regions.erase(region_idx);

  // // if any safe regions intersect new obstacles, mark them as invalid
  // for (int region_idx{0}; region_idx < m_safe_regions.size(); region_idx++) {
  //   for (auto const& obs : m_obstacle_add_queue) {
  //     if (m_safe_regions.at(region_idx).intersects(obs.second)) {
  //       m_invalid_regions.insert(region_idx);
  //       break;
  //     }
  //   }
  // }

  // update obstacle map and clear queues
  for (auto const& obs : m_obstacle_remove_queue)
    m_obstacle_map.erase(obs);
  for (auto const& obs : m_obstacle_add_queue)
    m_obstacle_map[obs.first] = obs.second;
  m_obstacle_add_queue.clear();
  m_obstacle_remove_queue.clear();
}


XYPoint IRIS2D::randomSeedPoint()
{
  return (XYPoint{});
}


//---------------------------------------------------------

void IRIS2D::setIRISProblem(const XYPoint &seed)
{
  std::string msg{"Cannot build region around x="};
  msg += doubleToStringX(seed.x()) + ", y=" + doubleToStringX(seed.y());
  msg += " because this point is inside an obstacle!";
  // make sure seed doesn't intersect any obstacles
  for (auto obs : m_obstacle_map) {
    if (obs.second.contains(seed)) {
      msg += " because this point is inside an obstacle!";
      reportRunWarning(msg);
      return;
    }
  }
  if (!m_iris_bounds.contains(seed)) {
    msg += " because this point is outside the bounds of the IRIS search!";
    reportRunWarning(msg);
    return;
  }

  // if seed is good, create a new IRIS problem
  m_current_problem = IRISProblem(seed, m_iris_bounds, m_max_iters, m_termination_threshold);
  for (auto obs : m_obstacle_map)
    m_current_problem.addObstacle(obs.second);
}


bool IRIS2D::runIRIS()
{
  // todo: where to catch iris errors?
  // todo: post stats on number of iterations, elapsed time
  if (m_current_problem.complete())
    return (true);

  // double num_iters{std::floor(m_max_iters/(GetAppFreq() * m_time_warp))};
  // return (m_current_problem.run(static_cast<int>(num_iters)));

  return (m_current_problem.run(m_max_iters + 1));  // while testing, run to completion
}


void IRIS2D::saveIRISRegion(int idx)
{
  // create poly and set visual params
  XYPolygon poly{m_current_problem.getPolygon()};
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
    poly.set_label(m_label_prefix + "_polygon_" + intToString(idx));
    m_safe_regions.at(idx) = poly;
  }

  // post polygon and visuals
  Notify(m_iris_region_var, poly.get_spec_pts_label(4));
  if (m_post_poly_visuals)
    Notify("VIEW_POLYGON", poly.get_spec());


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
    Notify("VIEW_POLYGON", ellipse.get_spec());
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
    // vars to subscribe to
    if (param == "obs_alert_var") {
      handled = setNonWhiteVarOnString(m_obs_alert_var, toupper(value));
    } else if ((param == "seed_point_var") || (param == "seed_pt_var")) {
      handled = setNonWhiteVarOnString(m_seed_pt_var, toupper(value));
    // publication config
    } else if (param == "label_prefix") {
      handled = setNonWhiteVarOnString(m_label_prefix, value);
    } else if (param == "post_polygons") {
      handled = setBooleanOnString(m_post_poly_visuals, value);
    } else if (param == "post_ellipses") {
      handled = setBooleanOnString(m_post_ellipse_visuals, value);
    // todo: remainder of visual params
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

  if (m_obs_alert_var.empty())
    m_obs_alert_var = "OBSTACLE_ALERT";
  if (m_seed_pt_var.empty())
    m_seed_pt_var = "IRIS_SEED_POINT";

  if (m_label_prefix.empty())
    m_label_prefix = GetAppName();

  // save IRIS region and post visuals if needed
  XYPolygon xy_iris_bounds{string2Poly(iris_bounds)};
  m_iris_bounds = IRISPolygon(xy_iris_bounds);
  if (m_post_poly_visuals) {
    xy_iris_bounds.set_label(m_label_prefix + "_bounds");
    xy_iris_bounds.set_color("edge", m_poly_edge_color);
    xy_iris_bounds.set_color("vertex", m_poly_vert_color);
    xy_iris_bounds.set_color("fill", "invisible");
    xy_iris_bounds.set_color("label", m_label_color);
    xy_iris_bounds.set_vertex_size(m_poly_vert_size * 2);
    xy_iris_bounds.set_edge_size(m_poly_edge_size * 2);
    xy_iris_bounds.set_transparency(1);
    Notify("VIEW_POLYGON", xy_iris_bounds.get_spec());
  }

  registerVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: registerVariables()

void IRIS2D::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("RUN_IRIS", 0);
  Register("CLEAR_IRIS", 0);
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

  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}
