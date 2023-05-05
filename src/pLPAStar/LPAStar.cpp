/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: LPAStar.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iostream>
#include <iterator>
#include <string>
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

  m_path_requested = false;
  m_transiting = false;
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

    if (key == "FOO")
      std::cout << "great!";

    else if (key != "APPCAST_REQ")  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
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
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
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
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool LPAStar::buildReport()
{
  using std::endl;
  return(true);
}




