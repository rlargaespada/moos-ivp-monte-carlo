/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ObsMonteCarloSim.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <cmath>
#include <cstdlib>
#include <iterator>
#include <string>
#include <vector>
#include "MBUtils.h"
#include "GeomUtils.h"
#include "ACTable.h"
#include "ObsMonteCarloSim.h"
#include "FileBuffer.h"
#include "ColorParse.h"
#include "XYFormatUtilsPoly.h"
#include "NodeRecordUtils.h"
#include "ObstacleFieldGenerator.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ObsMonteCarloSim::ObsMonteCarloSim()
{
  // Init Config variables
  m_min_range = 0;
  m_min_poly_size = 0;
  m_max_poly_size = 0;
  m_reuse_ids = true;
  m_label_prefix = "";

  m_poly_fill_color  = "white";
  m_poly_edge_color  = "gray50";
  m_poly_vert_color  = "gray50";
  m_poly_label_color = "invisible";

  m_poly_edge_size   = 1;
  m_poly_vert_size   = 1;
  m_poly_transparency = 0.15;

  m_draw_region = true;
  m_region_edge_color = "gray50";
  m_region_vert_color = "white";

  m_post_points = false;
  m_rate_points = 5;
  m_point_size  = 2;

  m_min_duration = -1;
  m_max_duration = -1;
  m_obs_refresh_interval = -1;

  m_reset_interval = -1;
  m_reset_range = 10;

  m_post_visuals = true;

  // Init State variables
  m_reset_tstamp   = 0;
  m_reset_request  = false;
  m_reset_pending  = false;
  m_newly_exited   = false;
  m_region_entered = false;
  m_reset_total    = 0;

  m_min_vrange_to_region = -1;

  m_obs_refresh_needed = false;
  m_obs_refresh_tstamp = -1;

  m_obstacles_posted = 0;
  m_obstacles_made   = 0;

  m_sensor_range = 50;
}

//---------------------------------------------------------
// Destructor

ObsMonteCarloSim::~ObsMonteCarloSim()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ObsMonteCarloSim::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p !=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    string sval = msg.GetString();


#if 0  // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif


    bool handled = true;
    if (key == "PMV_CONNECT") {
      m_obs_refresh_needed = true;
    } else if (key == "OBM_CONNECT") {
      m_obs_refresh_needed = true;
    } else if (key == "VEHICLE_CONNECT") {
      m_obs_refresh_needed = true;
    } else if (key == m_reset_var) {
      if (m_reset_var != "NONE")  // ignore if requests disabled
        m_reset_request = true;
    } else if (key == m_obstacle_file_var) {
      m_new_obstacle_file = sval;
    } else if (key == "UFOS_POINT_SIZE") {
      handled = handleMailPointSize(sval);
    } else if (key == "NODE_REPORT") {
      handled = handleMailNodeReport(sval);
    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
      handled = false;
    }

    if (!handled)
      reportRunWarning("Unhandled Mail: " + key + "=" + sval);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ObsMonteCarloSim::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ObsMonteCarloSim::Iterate()
{
  AppCastingMOOSApp::Iterate();

  updateVRanges();
  updateObstaclesField();
  updateObstaclesFromFile();
  updateObstaclesRefresh();

  if (m_post_points)
    postPoints();

  if (m_obs_refresh_needed)
    postObstaclesRefresh();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ObsMonteCarloSim::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  // Pass 1: Process everything but the obstacle file for now.
  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == "obstacle_file")
      handled = true;  // process later
    if (param == "obstacle_file_var")
      handled = setNonWhiteVarOnString(m_obstacle_file_var, value);
    else if (param == "label_prefix")
      handled = setNonWhiteVarOnString(m_label_prefix, value + "_");  // add trailing underscore
    else if ((param == "poly_vert_color") && isColor(value))
      handled = setColorOnString(m_poly_vert_color, value);
    else if ((param == "poly_fill_color") && isColor(value))
      handled = setColorOnString(m_poly_fill_color, value);
    else if ((param == "poly_edge_color") && isColor(value))
      handled = setColorOnString(m_poly_edge_color, value);
    else if ((param == "poly_label_color") && isColor(value))
      handled = setColorOnString(m_poly_label_color, value);

    else if (param == "poly_vert_size")
      handled = setNonNegDoubleOnString(m_poly_vert_size, value);
    else if (param == "poly_edge_size")
      handled = setNonNegDoubleOnString(m_poly_edge_size, value);
    else if (param == "poly_transparency")
      handled = setNonNegDoubleOnString(m_poly_transparency, value);

    else if (param == "draw_region")
      handled = setBooleanOnString(m_draw_region, value);
    else if ((param == "region_edge_color") && isColor(value))
      handled = setColorOnString(m_region_edge_color, value);
    else if ((param == "region_vert_color") && isColor(value))
      handled = setColorOnString(m_region_vert_color, value);

    else if (param == "post_points")
      handled = setBooleanOnString(m_post_points, value);
    else if (param == "rate_points")
      handled = setNonNegDoubleOnString(m_rate_points, value);
    else if (param == "point_size")
      handled = setNonNegDoubleOnString(m_point_size, value);

    else if (param == "sensor_range")
      handled = setNonNegDoubleOnString(m_sensor_range, value);

    else if (param == "min_duration")
      handled = handleConfigMinDuration(value);
    else if (param == "max_duration")
      handled = handleConfigMaxDuration(value);
    else if (param == "refresh_interval")
      handled = setNonNegDoubleOnString(m_obs_refresh_interval, value);

    else if (param == "reset_interval")
      handled = setNonNegDoubleOnString(m_reset_interval, value);
    else if (param == "reset_range")
      handled = setNonNegDoubleOnString(m_reset_range, value);
    else if (param == "reset_var")
      handled = setNonWhiteVarOnString(m_reset_var, toupper(value));
    else if (param == "reuse_ids")
      handled = setBooleanOnString(m_reuse_ids, value);

    else if (param == "post_visuals")
      handled = setBooleanOnString(m_post_visuals, value);

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }


  // m_reset_var is left unset in constructor because we don't
  // want to unnecessarily register for a variable that we don't
  // actually need
  if (m_reset_var.empty())
    m_reset_var = "UFOS_RESET";
  // same reasoning for m_obstacle_file_var
  if (m_obstacle_file_var.empty())
    m_obstacle_file_var = "NEW_OBSTACLE_FILE";

  // Pass 2: Process obstacle file last so all color settings can be
  // configured first, and applied as the obstacles are being created.
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = true;
    if (param == "obstacle_file")
      handled = handleConfigObstacleFile(value);
    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  m_reset_tstamp = MOOSTime();
  handleConfigObstacleDurations();
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ObsMonteCarloSim::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("PMV_CONNECT", 0);
  Register("OBM_CONNECT", 0);
  Register("VEHICLE_CONNECT", 0);
  if (!m_reset_var.empty() && (m_reset_var != "NONE"))
    Register(m_reset_var, 0);
  if (!m_obstacle_file_var.empty())
    Register(m_obstacle_file_var, 0);
  Register("UFOS_POINT_SIZE", 0);
  Register("NODE_REPORT", 0);
}

//------------------------------------------------------------
// Procedure: handleMailNodeReport()

bool ObsMonteCarloSim::handleMailNodeReport(string node_report)
{
  NodeRecord record = string2NodeRecord(node_report);
  if (!record.valid())
    return(false);
  string vname = record.getName();

  // If this is the first node report from this vehicle, consider
  // it also to be a query for obstacles.
  if (m_map_vrecords.count(vname) == 0)
    m_obs_refresh_needed = true;

  // Update the node record list for this vehicle
  m_map_vrecords[vname] = record;
  return(true);
}

//------------------------------------------------------------
// Procedure: handleMailPointSize()

bool ObsMonteCarloSim::handleMailPointSize(string str)
{
  str = tolower(str);
  double dval = atof(str.c_str());

  if (dval >= 1)
    m_point_size = dval;
  else if ((str == "smaller") && (m_point_size >=2))
    m_point_size -= 1;
  else if (str == "bigger")
    m_point_size += 1;
  else
    return(false);

  return(true);
}

//------------------------------------------------------------
// Procedure: handleConfigMinDuration()

bool ObsMonteCarloSim::handleConfigMinDuration(string sval)
{
  if (!isNumber(sval))
    return(false);

  m_min_duration = atof(sval.c_str());

  if (m_min_duration > m_max_duration)
    m_max_duration = m_min_duration;

  return(true);
}

//------------------------------------------------------------
// Procedure: handleConfigMaxDuration()

bool ObsMonteCarloSim::handleConfigMaxDuration(string sval)
{
  if (!isNumber(sval))
    return(false);

  m_max_duration = atof(sval.c_str());


  if (m_max_duration < m_min_duration)
    m_min_duration = m_max_duration;

  if ((m_min_duration < 0) && (m_max_duration >= 0))
    m_min_duration = 0;

  return(true);
}


//------------------------------------------------------------
// Procedure: handleConfigObstacleFile

bool ObsMonteCarloSim::handleConfigObstacleFile(string filename)
{
  vector<string> lines = fileBuffer(filename);
  unsigned int i, vsize = lines.size();
  if (vsize == 0) {
    reportConfigWarning("Error reading: " + filename);
    return(false);
  }

  for (i = 0; i < vsize; i++) {
    string line = stripBlankEnds(lines[i]);
    if ((line == "") || strBegins(line, "//"))
      continue;

    string left  = biteStringX(line, '=');
    string right = line;
    if (left == "region") {
      XYPolygon region = string2Poly(right);
      if (!region.is_convex()) {
        reportConfigWarning("Poorly specified region: " + right);
        return (false);
      }
      m_poly_region = region;
      m_poly_region.set_label(m_label_prefix + "obs_region");
      m_poly_region.set_color("edge", m_region_edge_color);
      m_poly_region.set_vertex_color(m_region_vert_color);
    } else if (left == "min_range") {
      bool ok = setNonNegDoubleOnString(m_min_range, right);
      if (!ok)
        reportConfigWarning("Poorly specified min_range: " + right);
    } else if (left == "min_size") {
      bool ok = setNonNegDoubleOnString(m_min_poly_size, right);
      if (!ok)
        reportConfigWarning("Poorly specified min_size: " + right);
    } else if (left == "max_size") {
      bool ok = setNonNegDoubleOnString(m_max_poly_size, right);
      if (!ok)
        reportConfigWarning("Poorly specified max_size: " + right);
    } else if (left == "poly") {
      XYPolygon poly = string2Poly(right);
      if (!poly.is_convex()) {
        reportConfigWarning("Poorly specified obstacle: " + right);
        return (false);
      }
      poly.set_label(m_label_prefix + poly.get_label());  // add prefix to label
      poly.set_color("edge", m_poly_edge_color);
      poly.set_color("vertex", m_poly_vert_color);
      poly.set_color("fill", m_poly_fill_color);
      poly.set_color("label", m_poly_label_color);
      poly.set_vertex_size(m_poly_vert_size);
      poly.set_edge_size(m_poly_edge_size);
      poly.set_transparency(m_poly_transparency);
      m_obstacles.push_back(poly);
      m_durations.push_back(-1);
      m_obstacles_made++;
    }
  }

  // Further Sanity checks after processing the whole file

  if (m_min_poly_size == 0) {
    reportConfigWarning("Unset or non-positive min_poly_size");
    return(false);
  }
  if (m_max_poly_size == 0) {
    reportConfigWarning("Unset max_poly_size");
    return(false);
  }
  if (m_min_range == 0) {
    reportConfigWarning("Unset or non-positive min_range");
    return(false);
  }
  if (m_min_poly_size > m_max_poly_size) {
    m_min_poly_size = m_max_poly_size;
    reportConfigWarning("min_poly_size greater than max_poly_size");
    return(false);
  }

  return(true);
}

//------------------------------------------------------------
// Procedure: handleConfigObstacleDurations()

void ObsMonteCarloSim::handleConfigObstacleDurations()
{
  if (m_max_duration <= 0)
    return;

  double min_duration = m_min_duration;
  if (min_duration < 0)
    min_duration = 0;

  for (unsigned int i = 0; i < m_durations.size(); i++)
    m_durations[i] = randomDouble(min_duration, m_max_duration);
}

//------------------------------------------------------------
// Procedure: updateVRanges()

void ObsMonteCarloSim::updateVRanges()
{
  // Part 1: calculate the new min_vrange_to_region
  double min_vrange = -1;
  map<string, NodeRecord>::iterator p;
  for (p = m_map_vrecords.begin(); p != m_map_vrecords.end(); p++) {
    string     vname = p->first;
    NodeRecord record = p->second;

    double vx = record.getX();
    double vy = record.getY();

    if (m_poly_region.is_convex()) {
      if (!m_poly_region.contains(vx, vy)) {
        double range = m_poly_region.dist_to_poly(vx, vy);
        m_map_vrange[vname] = range;
        if ((min_vrange < 0) || (range < min_vrange))
          min_vrange = range;
      } else {
        m_region_entered = true;
        min_vrange = 0;
      }
    }
  }

  // Part 2: Determine if vehicles are newly_exited
  if ((m_min_vrange_to_region <= 0) && (min_vrange > 0))
    m_newly_exited = true;
  if ((min_vrange <= 0) || !m_region_entered)
    m_newly_exited = false;

  m_min_vrange_to_region = min_vrange;

  // Part 3: If resetting is enabled, determine if reset warranted
  if ((m_reset_interval > 0) || m_reset_request) {
    if (m_min_vrange_to_region > m_reset_range) {
      if (m_newly_exited || m_reset_request) {
        double elapsed = m_curr_time - m_reset_tstamp;
        if ((elapsed > m_reset_interval) || m_reset_request) {
          m_reset_pending = true;
          m_reset_request = false;
          m_reset_tstamp = m_curr_time;
          m_newly_exited = false;
        }
      }
    }
  }
}


//------------------------------------------------------------
// Procedure: updateObstaclesField()
//   Purpose: If the reset_interval is non-negative, then check
//            if enough time has elapsed for a reset. If so, then
//            check if a reset is warranted based on range, and
//            then reset the obstacles.

void ObsMonteCarloSim::updateObstaclesField()
{
  if (!m_reset_pending)
    return;

  Notify("KNOWN_OBSTACLE_CLEAR", GetAppName());

  // Seed randomness from fractional part of current moos time
  double tmp;
  double moos_time_frac{modf(m_curr_time, &tmp)};  // get fraction from current time
  int seed{static_cast<int>(moos_time_frac * pow(10, 6))};  // scale frac and cast to int
  srand(seed);

  // Do the obstacle regeneration
  vector<XYPolygon> new_obstacles;
  bool ok = true;
  for (unsigned int i=0; (ok && (i < m_obstacles.size())); i++) {
    ok = ok && generateObstacle(&new_obstacles, 1000);
  }

  // Sanity check the results.
  if (new_obstacles.size() != m_obstacles.size())
    return;

  // If new obstacles will have a new id/label, erase obstacles
  // before overwriting them with new ones with different labels
  if (!m_reuse_ids)
    postObstaclesErase();

  m_obstacles = new_obstacles;
  m_obstacles_made += new_obstacles.size();
  m_reset_total++;


  // Apply the local simulator viewing preferences
  for (unsigned int i=0; i < m_obstacles.size(); i++) {
    m_obstacles[i].set_color("edge", m_poly_edge_color);
    m_obstacles[i].set_color("vertex", m_poly_vert_color);
    m_obstacles[i].set_color("fill", m_poly_fill_color);
    m_obstacles[i].set_color("label", m_poly_label_color);
    m_obstacles[i].set_vertex_size(m_poly_vert_size);
    m_obstacles[i].set_edge_size(m_poly_edge_size);
    m_obstacles[i].set_transparency(m_poly_transparency);
  }

  m_obs_refresh_needed = true;
  m_reset_pending = false;
}

// ------------------------------------------------------------
bool ObsMonteCarloSim::generateObstacle(vector<XYPolygon>* obs_vec, unsigned int tries)
{
  if (!m_poly_region.is_convex())
    return (false);

  double minx = m_poly_region.get_min_x();
  double miny = m_poly_region.get_min_y();
  double maxx = m_poly_region.get_max_x();
  double maxy = m_poly_region.get_max_y();

  double xlen = maxx - minx;
  double ylen = maxy - miny;

  double radius = m_min_poly_size;
  if (m_max_poly_size > m_min_poly_size) {
    int rand_int_r = rand() % 1000;
    double rand_pct_r = static_cast<double>(rand_int_r) / 1000;
    radius = m_min_poly_size;
    radius += ((m_max_poly_size - m_min_poly_size) * rand_pct_r);
  }

  for (unsigned int k=0; k < tries; k++) {
    int rand_int_x = rand() % 10000;
    int rand_int_y = rand() % 10000;

    double rand_pct_x = static_cast<double>(rand_int_x) / 10000;
    double rand_pct_y = static_cast<double>(rand_int_y) / 10000;

    double rand_x = minx + (rand_pct_x * xlen);
    double rand_y = miny + (rand_pct_y * ylen);

    // Reject if poly center point is not in the overall region
    if (!m_poly_region.contains(rand_x, rand_y))
      continue;

    // Reject if poly center point is in an existing obstacle
    bool pt_in_obstacle = false;
    for (unsigned int i=0; i < obs_vec->size(); i++) {
      if ((*obs_vec)[i].contains(rand_x, rand_y)) {
        pt_in_obstacle = true;
      }
    }
    if (pt_in_obstacle)
      continue;

    // "radial:: x=val, y=val, radius=val, pts=val, snap=val, label=val"
    string str = "format=radial, x=" + doubleToString(rand_x, 1);
    str += ", y=" + doubleToString(rand_y, 1);
    str += ",radius=" + doubleToString(radius);
    str += ",snap=0.1";  // + doubleToStringX(0.1, 3);  // 0.1m precision
    str += ",pts=8";  // + uintToString(m_poly_vertices);  //  make octagons
    unsigned int begin_id{(m_reuse_ids) ? 0 : m_obstacles_made};
    str += ",label=" + m_label_prefix + "ob_" + uintToString(begin_id + obs_vec->size());

    XYPolygon try_poly = string2Poly(str);
    if (!try_poly.is_convex()) {
      // cout << "     try_poly is not convex." << endl;
      return (false);
    }

    // Reject if poly is not in the overall region
    if (!m_poly_region.contains(try_poly))
      continue;

    // Reject if poly intersects any existing obstacle
    bool poly_ints_obstacle = false;
    for (unsigned int i=0; i < obs_vec->size(); i++) {
      if ((*obs_vec)[i].intersects(try_poly)) {
        poly_ints_obstacle = true;
      }
    }
    if (poly_ints_obstacle)
      continue;

    // Reject if poly is too close to any existing obstacle
    bool poly_closeto_obstacle = false;
    for (unsigned int i=0; i < obs_vec->size(); i++) {
      if ((*obs_vec)[i].dist_to_poly(try_poly) < m_min_range) {
        poly_closeto_obstacle = true;
      }
    }
    if (poly_closeto_obstacle)
      continue;

    // Success!!!
    obs_vec->push_back(try_poly);
    return (true);
  }
  return (false);
}


//------------------------------------------------------------
// Procedure: updateObstaclesFromFile()
void ObsMonteCarloSim::updateObstaclesFromFile()
{
  if (m_new_obstacle_file.empty())
    return;

  // clear existing obstacles
  Notify("KNOWN_OBSTACLE_CLEAR", GetAppName());
  postObstaclesErase();
  m_obstacles.clear();

  // reuse config handling function to update obstacle field
  handleConfigObstacleFile(m_new_obstacle_file);

  // set up to post new obstacles
  m_new_obstacle_file.clear();
  m_reset_total++;
  m_obs_refresh_needed = true;
}



//------------------------------------------------------------
// Procedure: updateObstaclesRefresh()
//   Purpose: If obstacles are set to be refreshed periodically,
//            check if refresh is needed now based on elapsed time.

void ObsMonteCarloSim::updateObstaclesRefresh()
{
  if (m_obs_refresh_interval > 0) {
    double elapsed = m_curr_time - m_obs_refresh_tstamp;
    if (elapsed > m_obs_refresh_interval)
      m_obs_refresh_needed = true;
  }
}


//------------------------------------------------------------
// Procedure: postObstaclesRefresh()
//     Notes: o VIEW_POLYGON info is likely for GUI apps like PMV
//     Notes: o The KNOWN_OBSTACLE is intended for the benefit of
//              other shoreside apps, e.g., uFldCollObDetect so it
//              has access to ground truth. Thus KNOWN_OBSTACLE is
//              not intented to be shared out to the vehicles
//            o The GIVEN_OBSTACLE is intended for sharing to the
//              vehicles, only if this sim is not in "post_points"
//              mode. In post_points mode, this sim is sharing
//              simulated sensor data, not actual obstacle info

void ObsMonteCarloSim::postObstaclesRefresh()
{
  // =================================================
  // Part 1: Post the viewable info
  // =================================================
  if (m_post_visuals) {
    for (unsigned int i=0; i < m_obstacles.size(); i++) {
      string spec = m_obstacles[i].get_spec(2);
      Notify("VIEW_POLYGON", spec);
    }
    if (m_draw_region && m_poly_region.is_convex())
      Notify("VIEW_POLYGON", m_poly_region.get_spec());
  }


  // =================================================
  // Part 2: Post ground truth
  // =================================================
  for (unsigned int i=0; i < m_obstacles.size(); i++) {
    string spec = m_obstacles[i].get_spec_pts_label(2);
    string key  = m_obstacles[i].get_label();
    if (m_durations[i] >= 0)
      spec += ",duration=" + doubleToStringX(m_durations[i]);

    Notify("KNOWN_OBSTACLE", spec);
    if (!m_post_points) {
      Notify("GIVEN_OBSTACLE", spec);
      m_map_giv_published[key]++;
    }
  }

  m_obs_refresh_needed = false;
  m_obs_refresh_tstamp = m_curr_time;
  m_obstacles_posted++;
}

//------------------------------------------------------------
// Procedure: postObstaclesErase()

void ObsMonteCarloSim::postObstaclesErase()
{
  for (unsigned int i=0; i < m_obstacles.size(); i++) {
    XYPolygon obstacle = m_obstacles[i];
    obstacle.set_duration(0);
    string spec = obstacle.get_spec_inactive();
    if (m_post_visuals)
      Notify("VIEW_POLYGON", spec);
    Notify("KNOWN_OBSTACLE", spec);
    if (!m_post_points)
      Notify("GIVEN_OBSTACLE", spec);
  }
}


//------------------------------------------------------------
// Procedure: postPoints()
//      Note: Points are published as:
//            TRACKED_FEATURE = x=5,y=8,label=key,size=4,color=1

void ObsMonteCarloSim::postPoints()
{
#if 1
  map<string, NodeRecord>::iterator p;
  for (p = m_map_vrecords.begin(); p != m_map_vrecords.end(); p++) {
    string vname  = p->first;
    string uvname = toupper(p->first);
    double osx = p->second.getX();
    double osy = p->second.getY();
    string vcolor = p->second.getColor("yellow");

    for (unsigned int i=0; i < m_obstacles.size(); i++) {
      if (m_obstacles[i].dist_to_poly(osx, osy) <= m_sensor_range) {
        for (unsigned int j=0; j < m_rate_points; j++) {
          double x, y;
          bool ok = randPointOnPoly(osx, osy, m_obstacles[i], x, y);
          if (ok) {
            string key = m_obstacles[i].get_label();
            string msg = "x=" + doubleToStringX(x, 2);
            msg += ",y=" + doubleToStringX(y, 2);
            msg += ",key=" + key;
            Notify("TRACKED_FEATURE_"+uvname, msg);
            m_map_pts_published[key]++;
            int label_index = static_cast<int>(m_map_pts_published[key]) % 100;

            if (m_post_visuals) {
              XYPoint p(x, y);
              p.set_label(vname + ":" + key + ":" + intToString(label_index));
              p.set_vertex_color(vcolor);
              p.set_vertex_size(m_point_size);
              p.set_label_color("invisible");
              p.set_duration(10);
              string spec = p.get_spec();
              Notify("VIEW_POINT", spec);
            }
          }
        }
      }
    }
  }
#endif
#if 0
  for (unsigned int i=0; i < m_obstacles.size(); i++) {
    for (unsigned int j=0; j < m_rate_points; j++) {
      double x, y;
      bool ok = randPointInPoly(m_obstacles[i], x, y);
      if (ok) {
        string key = m_obstacles[i].get_label();
        string msg = "x=" + doubleToStringX(x, 2);
        msg += ",y=" + doubleToStringX(y, 2);
        msg += ",key=" + key;
        Notify("TRACKED_FEATURE", msg);
        reportEvent("TRACKED_FEATURE="+msg);
        m_map_pts_published[key]++;
      }
    }
  }
#endif
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ObsMonteCarloSim::buildReport()
{
  string min_dur_str = "n/a";
  if (m_min_duration > 0)
    min_dur_str = doubleToString(m_min_duration, 1);

  string max_dur_str = "n/a";
  if (m_max_duration > 0)
    max_dur_str = doubleToString(m_max_duration, 1);

  string refresh_interval_str = "n/a";
  if (m_obs_refresh_interval > 0)
    refresh_interval_str = doubleToStringX(m_obs_refresh_interval, 1);

  m_msgs << "================================" << endl;
  m_msgs << "Config (Obstacles) " << endl;
  m_msgs << "  Obstacles: " << uintToString(m_obstacles.size()) << endl;
  m_msgs << "  MinRange:  " << doubleToStringX(m_min_range)     << endl;
  m_msgs << "  MinSize:   " << doubleToStringX(m_min_poly_size) << endl;
  m_msgs << "  MaxSize:   " << doubleToStringX(m_max_poly_size) << endl;
  m_msgs << "  ReuseIDs:  " << boolToString(m_reuse_ids) << endl;
  m_msgs << "Config (Points)  " << endl;
  m_msgs << "  Post Points:   " << boolToString(m_post_points) << endl;
  m_msgs << "  Rate Points:   " << doubleToStringX(m_rate_points) << endl;
  m_msgs << "Config (Duration)" << endl;
  m_msgs << "  Min Duration:  " << min_dur_str << endl;
  m_msgs << "  Max Duration:  " << max_dur_str << endl;
  m_msgs << "  Refresh Intrv: " << refresh_interval_str << endl;
  m_msgs << "Config (Reset)   " << endl;
  if (m_reset_var != "NONE")
    m_msgs << "  Reset Var:     " << m_reset_var << endl;
  else
    m_msgs << "  RESET REQUESTS DISABLED" << endl;
  m_msgs << "  Reset Range:   " << doubleToString(m_reset_range, 0) << endl;
  m_msgs << "  Reset_Interv:  " << doubleToString(m_reset_interval, 0) << endl;
  m_msgs << "================================" << endl;
  m_msgs << "State (Obstacles)               " << endl;
  m_msgs << "  Obstacles Posted: " << uintToString(m_obstacles_posted) << endl;
  m_msgs << "State (resetting)   " << endl;
  m_msgs << "  Min Poly Range: " << doubleToString(m_min_vrange_to_region, 0) << endl;
  m_msgs << "  Reset Pending:  " << boolToString(m_reset_pending) << endl;
  m_msgs << "  Newly Exited :  " << boolToString(m_newly_exited) << endl;
  m_msgs << "  Reset Tstamp :  " << doubleToString(m_reset_tstamp, 0) << endl;
  m_msgs << "  Reset Total  :  " << uintToString(m_reset_total) << endl;

  m_msgs << endl << endl;

  ACTable actab(4);
  actab << "Obs | Obs      | Points     | Given ";
  actab << "Key | Duration | Published  | Published";
  actab.addHeaderLines();

  for (unsigned int i=0; i < m_obstacles.size(); i++)  {
    string key = m_obstacles[i].get_label();
    string dur_str = doubleToString(m_durations[i], 1);
    string pts_str = uintToString(m_map_pts_published[key]);
    string giv_str = uintToString(m_map_giv_published[key]);

    actab << key;
    actab << dur_str;
    actab << pts_str;
    actab << giv_str;
  }

  m_msgs << actab.getFormattedString();
  m_msgs << endl << endl;

  return(true);
}
