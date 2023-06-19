/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IRIS.cpp                                        */
/*    DATE: June 14th, 2023                                 */
/************************************************************/

#include<Eigen/Dense>
#include <iterator>
#include <string>
#include "MBUtils.h"
#include "ACTable.h"
#include "IRIS.h"

//---------------------------------------------------------
// Constructor()

IRIS::IRIS()
{
  //* Config Variables
  // vars to subscribe to, all are set in onStartup();
  m_obs_alert_var = "";
  m_seed_pt_var = "";

  // publication config
  m_iris_region_var = "IRIS_REGION";
  m_complete_var = "IRIS_COMPLETE";
  m_post_visuals = true;

  // IRIS config
  m_mode = "manual";
  m_desired_regions = 20;
  m_max_iters = 100;

  //* State Variables
  m_clear_pending = false;
  m_run_pending = false;
  m_iris_active = false;
}

//---------------------------------------------------------
// Destructor

IRIS::~IRIS()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool IRIS::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key    = msg.GetKey();

#if 0  // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    std::string sval  = msg.GetString();
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if (key == "FOO")
      std::cout << "great!";

    else if (key != "APPCAST_REQ")  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool IRIS::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool IRIS::Iterate()
{
  AppCastingMOOSApp::Iterate();

  /*
  handleRequests()  //* handles iris active/inactive, clearing obstacles
    if clear pending
      erase all regions
      if not in auto, mark inactive
    else if run pending
      if manual and already active, mark refresh or something
      mark active (doesn't affect auto, leave comment)
    mark clear pending false
    mark run pending false

  syncObstacles()
    if queues are empty, return
    for remove queue, invalidate any regions which intersect obs
    for add queue, invalidate any regions which intersect obs
    if it doesn't intersect with any obstacles, remove from invalid queue //? return early conflicts?
    add obs to map and clear queues

  if seed point queue isn't empty
    buildRegion(seed point)
  else if active and still regions to go
    buildRegion(random seed)
    if manual and last region
      mark inactive
  else if active and invalidate queue isn't empty
    erase region in invalidate queue
    buildRegion(center of removed region)

  buildRegion(seed point)
    run IRIS around that seed point using obstacle map


  first pass at IRIS: when posting to seed point, build region
  */

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool IRIS::OnStartUp()
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
    } else if (param == "seed_point_var") {
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

void IRIS::registerVariables()
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

bool IRIS::buildReport()
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
