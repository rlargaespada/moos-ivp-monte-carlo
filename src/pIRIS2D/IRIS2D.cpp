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
#include <iterator>
#include <string>
#include "ACTable.h"
#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsPoint.h"
#include "XYPolygon.h"
#include "XYPoint.h"
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
  m_post_visuals = true;
  m_label_color = "white";
  m_poly_fill_color = "violet";
  m_poly_edge_color = "blueviolet";
  m_poly_vert_color = "blueviolet";

  m_poly_edge_size = 1;
  m_poly_vert_size = 1;
  m_poly_transparency = 0.15;

  m_ellipse_fill_color = "invisible";
  m_ellipse_edge_color = "slateblue";

  m_ellipse_edge_size = 1;
  m_ellipse_transparency = 0.15;

  // IRIS config
  m_mode = "manual";
  m_desired_regions = 20;
  m_max_iters = 100;
  m_termination_threshold = 2e-2;

  //* State Variables
  m_clear_pending = false;
  m_run_pending = false;
  m_iris_active = false;

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
  handleRequests()  //* handles iris active/inactive, clearing obstacles
    if clear pending
      erase all regions
      if not in auto, mark inactive
    else if run pending
      mark active (doesn't affect auto, leave comment)
    mark clear pending false
    mark run pending false

  syncObstacles()
    if queues are empty, return
    for remove queue, invalidate any regions which intersect obs
    for add queue, invalidate any regions which intersect obs
    if it doesn't intersect with any obstacles, remove from invalid queue
    add obs to map and clear queues

  if seed point queue isn't empty
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

  if (!m_seed_pt_queue.empty()) {
    buildRegion(m_seed_pt_queue.front());
    m_seed_pt_queue.pop();
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------
void IRIS2D::handleRequests()
{
  if (m_clear_pending) {
    // todo: erase all regions
    if (m_mode != "auto")
      m_iris_active = false;
  } else if (m_run_pending) {
    m_iris_active = true;  // always true in auto mode
  }

  m_clear_pending = false;
  m_run_pending = false;
}


void IRIS2D::syncObstacles()
{}


XYPoint IRIS2D::randomSeedPoint()
{
  return (XYPoint{});
}


bool IRIS2D::buildRegion(const XYPoint &seed)
{
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
    } else if (param == "post_visuals") {
      handled = setBooleanOnString(m_post_visuals, value);
    // IRIS config
    } else if (param == "mode") {
      handled = true;  // todo
    } else if (param == "desired_regions") {
      handled = setPosUIntOnString(m_desired_regions, value);
    } else if (param == "iris_bounds") {
      handled = true;  // todo
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
