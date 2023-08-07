/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GCS2D.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <algorithm>
#include <iterator>
#include <vector>
#include "ACTable.h"
#include "GeomUtils.h"
#include "MacroUtils.h"
#include "MBUtils.h"
#include "VarDataPair.h"
#include "VarDataPairUtils.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "GCS2D.h"


//---------------------------------------------------------
// Constructor()

GCS2D::GCS2D()
{
  //* Config Variables
  // vars to subscribe to, all are set in onStartUp()
  m_path_request_var = "";
  m_wpt_complete_var = "";

  // IRIS interface
  m_iris_file = "";  // default: unset
  m_run_iris_on_new_path = false;  // default: false
  m_run_iris_var = "RUN_IRIS";  // default: RUN_IRIS
  m_clear_iris_var;  // default: CLEAR_IRIS, set in onStartUp
  m_iris_region_var;  // default: IRIS_REGION, set in onStartUp
  m_iris_complete_var;  // default: IRIS_COMPLETE, set in onStartUp

  // publication config
  m_prefix = "";

  m_path_found_var = "PATH_FOUND";
  m_path_complete_var = "PATH_COMPLETE";
  m_path_stats_var = "PATH_STATS";
  m_path_failed_var = "PATH_FAILED";

  // GCS config
  m_bezier_order = 1;
  m_bezier_continuity_req = 0;
  m_path_length_weight = 1;
  m_derivative_regularization_weight = 0;
  m_derivative_regularization_order = 0;

  //* State Variables
  // planning state data
  m_mode = PlannerMode::IDLE;
  m_planning_start_time = 0;
  m_planning_end_time = 0;
  m_path_len_traversed = 0;
  m_next_path_idx = 0;
  m_gcs_step = GCSStep::BUILD_GRAPH;
}


//---------------------------------------------------------
// Destructor

GCS2D::~GCS2D()
{
}


//---------------------------------------------------------
// Procedure: OnNewMail()

bool GCS2D::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p=NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key = msg.GetKey();

#if 0  // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

      // generic path planning interface
    if (key == m_path_request_var) {
      std::string request{msg.GetString()};
      reportEvent("Path Requested: " + request);
      if (setEndpoints(request))
        m_mode = PlannerMode::REQUEST_PENDING;
      else
        reportRunWarning("Invalid " + key + ": " + request);
    } else if (key == m_wpt_complete_var) {
      if (m_mode == PlannerMode::IN_TRANSIT) {
        m_mode = PlannerMode::PATH_COMPLETE;

        // add final segment of path to dist traveled
        handleNewWpt(m_path.size() - 1);
      }
    } else if (key == "WPT_INDEX") {
      // add just finished segment of path to dist traveled
      if (m_mode == PlannerMode::IN_TRANSIT)
        handleNewWpt(static_cast<int>(msg.GetDouble()));
    } else if ((key == "NODE_REPORT_LOCAL") || (key == "NODE_REPORT")) {
      std::string report{tolower(msg.GetString())};
      double xval{tokDoubleParse(report, "x", ',', '=')};
      double yval{tokDoubleParse(report, "y", ',', '=')};
      m_vpos.set_vertex(xval, yval);

    // pIRIS2D interface
    } else if (key == m_clear_iris_var) {
      clearIRISRegions();
    } else if (key == m_iris_region_var) {
      newIRISRegion(msg.GetString());
    } else if (key == m_iris_complete_var) {
      if (m_mode == PlannerMode::IRIS_IN_PROGRESS)
        m_mode = PlannerMode::GCS_IN_PROGRESS;

    } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return (true);
}


bool GCS2D::setEndpoints(const std::string& request)
{
  std::string start, goal;
  std::string request_lower{tolower(request)};

  // pull out start and goal, return fail if can't find
  start = tokStringParse(request_lower, "start", ';', '=');
  goal = tokStringParse(request_lower, "goal", ';', '=');
  if ((start.empty()) || (goal.empty()))
    return (false);

  // apply spec to start and goal, if parsing fails return fail
  m_start_point = string2Point(start);
  m_goal_point = string2Point(goal);
  if (!m_start_point.valid() || !m_goal_point.valid())
    return (false);

  return (true);
}


void GCS2D::handleNewWpt(int new_wpt_idx)
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


void GCS2D::newIRISRegion(const std::string& spec)
{
  XYPolygon region{string2Poly(spec)};
  if (region.is_convex()) {
    if (region.get_label().empty())  // add a label if one's not supplied
      region.set_label("region_" + std::to_string(m_safe_regions.size()));
    m_safe_regions.push_back(region);
  } else {
    reportRunWarning("Invalid IRIS region: " + spec);
  }
}


//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GCS2D::OnConnectToServer()
{
  registerVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GCS2D::Iterate()
{
  AppCastingMOOSApp::Iterate();

  bool path_found{false};

  switch (m_mode) {
    case (PlannerMode::REQUEST_PENDING):  // new path request
      // post new plan messages
      Notify(m_prefix + m_path_found_var, "false");
      Notify(m_prefix + m_path_complete_var, "false");
      Notify(m_prefix + m_path_failed_var, "false");
      postFlags(m_init_plan_flags);

      // clear planning state data
      m_planning_start_time = MOOSTime();
      m_path_len_traversed = 0;
      m_path.clear();

      // begin building GCS
      m_gcs_step = GCSStep::BUILD_GRAPH;  // go back to first gcs step
      path_found = planPath();
      break;
    // if already planning a path, pick up from where we left off
    case (PlannerMode::GCS_IN_PROGRESS):
      path_found = planPath();
      break;
    case (PlannerMode::PATH_COMPLETE):  // if path is complete, cleanup
      Notify(m_prefix + m_path_complete_var, "true");
      postFlags(m_end_flags);
      m_mode = PlannerMode::IDLE;
      break;
    default:  // IDLE, IRIS_IN_PROGRESS, PLANNING_FAILED, IN_TRANSIT
      // todo: implement replanning while IN_TRANSIT
      // do nothing in these modes, waiting for mail from other apps
      break;
  }

  // notify that path has been found
  if (path_found) {
    m_planning_end_time = MOOSTime();
    m_mode = PlannerMode::IN_TRANSIT;
    postPath();
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------

void GCS2D::clearIRISRegions()
{
  m_safe_regions.clear();
  m_iris_file.clear();  // not using this anymore if IRIS is cleared
}


bool GCS2D::requestIRISRegions()
{
  // todo: make sure there's a pIRIS2D app running, otherwise report run warning and return false
  return (Notify(m_run_iris_var, "now"));
}


bool GCS2D::buildGraph()
{
  m_gcs = std::unique_ptr<GraphOfConvexSets> (new GraphOfConvexSets(
    m_safe_regions,
    m_bezier_order,
    m_bezier_continuity_req,
    m_start_point,
    m_goal_point,
    m_options));
  return (true);
}


bool GCS2D::populateModel()
{
  m_gcs->addContinuityConstraints();
  m_gcs->addPathLengthCost(m_path_length_weight);
  m_gcs->populateModel();
  return (true);
}


bool GCS2D::planPath()
{
  bool path_found{false};

  switch (m_mode) {
    // initial request to start planning
    case (PlannerMode::REQUEST_PENDING):
      // check if we need to run IRIS
      if ((m_run_iris_on_new_path) || (m_safe_regions.empty())) {
        if (requestIRISRegions()) {
          m_mode = PlannerMode::IRIS_IN_PROGRESS;
          m_gcs_step = GCSStep::BUILD_GRAPH;  // force gcs back to first step
        } else {
          handlePlanningFail("Failed to request IRIS regions!");
          m_gcs_step = GCSStep::FAILED;
        }

      // if we don't need to run IRIS, start by building graph
      } else {
        // if graph was built successfully, move onto the next step next iteration
        if (buildGraph()) {
          m_mode = PlannerMode::GCS_IN_PROGRESS;
            // m_gcs_step = GCSStep::PREPROCESS_GRAPH;  // todo: implement preprocessing
          m_gcs_step = GCSStep::POPULATE_MODEL;
        // otherwise report failure
        } else {
          handlePlanningFail("Failed to build GCS!");
          m_gcs_step = GCSStep::FAILED;
        }
      }

      break;

    // subsequent requests to continue planning
    case (PlannerMode::GCS_IN_PROGRESS):
      switch (m_gcs_step) {
        case (GCSStep::BUILD_GRAPH):
          // if graph was built successfully, move onto the next step next iteration
          if (buildGraph()) {
            // m_gcs_step = GCSStep::PREPROCESS_GRAPH;  // todo: implement preprocessing
            m_gcs_step = GCSStep::POPULATE_MODEL;
          // otherwise report failure
          } else {
            handlePlanningFail("Failed to build GCS!");
            m_gcs_step = GCSStep::FAILED;
          }
          break;

        case (GCSStep::POPULATE_MODEL):
          // if model was populated successfully, move onto the next step next iteration
          if (populateModel()) {
            m_gcs_step = GCSStep::START_MOSEK;
          // otherwise report failure
          } else {
            handlePlanningFail("Failed to populate MOSEK model!");
            m_gcs_step = GCSStep::FAILED;
          }
          break;

        case (GCSStep::START_MOSEK):
          // if preconditions are met, start MOSEK optimization
          if (!checkPlanningPreconditions()) {
            handlePlanningFail();  // checkPlanningPreconditions posts its own warnings
            m_gcs_step = GCSStep::FAILED;
          } else {
            // todo: start mosek in separate thread
            m_gcs_step = GCSStep::MOSEK_RUNNING;
          }
          break;

        case (GCSStep::MOSEK_RUNNING):
          // todo: check if mosek is done or not
          // todo: if it's not done, increment some counter
          // todo: if it's done, parse out path, return true if it succeeds
          // todo: implement convex rounding

          // placeholders while testing
          path_found = true;
          m_path.add_vertex(m_start_point);
          m_path.add_vertex(m_goal_point);
          m_gcs_step = GCSStep::COMPLETE;
          break;
        default:  // shouldn't get here
          break;
      }

      break;

    // in all other modes, do nothing
    default:
      break;
  }

  return (path_found);
}


//---------------------------------------------------------

bool GCS2D::checkPlanningPreconditions()
{
  return (true);
}


void GCS2D::handlePlanningFail(const std::string& warning_msg)
{
  if (!warning_msg.empty()) reportRunWarning(warning_msg);
  reportRunWarning("Planning failed!");
  m_mode = PlannerMode::PLANNING_FAILED;
  Notify(m_prefix + m_path_failed_var, "true");
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GCS2D::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p != sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    // vars to subscribe to
    if (param == "path_request_var") {
      handled = setNonWhiteVarOnString(m_path_request_var, toupper(value));
    } else if (param == "wpt_complete_var") {
      handled = setNonWhiteVarOnString(m_wpt_complete_var, toupper(value));
    // publication config
    } else if (param == "prefix") {
      handled = setNonWhiteVarOnString(m_prefix, toupper(value));
    } else if ((param == "init_plan_flag") || param == "initflag") {
      handled = addVarDataPairOnString(m_init_plan_flags, value);
    } else if ((param == "traverse_flag") || param == "traverseflag") {
      handled = addVarDataPairOnString(m_traverse_flags, value);
    } else if ((param == "end_flag") || (param == "endflag")) {
      handled = addVarDataPairOnString(m_end_flags, value);
    // pIRIS2D interface config
    } else if (param == "iris_file") {
      handled = setNonWhiteVarOnString(m_iris_file, value);
    } else if (param == "rerun_iris_on_path_request") {
      handled = setBooleanOnString(m_run_iris_on_new_path, value);
    } else if (param == "iris_run_var") {
      handled = setNonWhiteVarOnString(m_run_iris_var, toupper(value));
    } else if (param == "iris_clear_var") {
      handled = setNonWhiteVarOnString(m_clear_iris_var, toupper(value));
    } else if (param == "iris_region_var") {
      handled = setNonWhiteVarOnString(m_iris_region_var, toupper(value));
    } else if (param == "iris_complete_var") {
      handled = setNonWhiteVarOnString(m_iris_complete_var, toupper(value));
    // GCS config
    } else if (param == "bezier_order") {
      handled = setIntOnString(m_bezier_order, value);
    } else if (param == "bezier_continuity") {
      handled = setIntOnString(m_bezier_continuity_req, value);
    } else if (param == "path_length_weight") {
      handled = setNonNegDoubleOnString(m_path_length_weight, value);
    } else if (param == "derivative_regularization_weight") {
      handled = setNonNegDoubleOnString(m_derivative_regularization_weight, value);
    } else if (param == "derivative_regularization_order") {
      handled = setIntOnString(m_derivative_regularization_order, value);
    // optimization options
    } else if (param == "relaxation") {
      handled = setBooleanOnString(m_options.convex_relaxation, value);
    } else if (param == "max_rounded_paths") {
      handled = setUIntOnString(m_options.max_rounded_paths, value);
    } else if (param == "preprocess_graph") {
      handled = setBooleanOnString(m_options.preprocessing, value);
    } else if (param == "max_rounded_trials") {
      handled = setUIntOnString(m_options.max_rounding_trials, value);
    } else if (param == "flow_tolerance") {
      handled = setNonNegDoubleOnString(m_options.flow_tolerance, value);
    } else if (param == "rounding_seed") {
      handled = setIntOnString(m_options.rounding_seed, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // additional config checks and handling
  if (!m_iris_file.empty()) {
    if (!readRegionsFromFile()) {
      reportConfigWarning("Could not read safe regions from " + m_iris_file);
      m_iris_file.clear();  // clear to force use of pIRIS2D
    } else {
    m_run_iris_on_new_path = false;  // if file was ok, force this to false
    }
  }
  checkConfigAssertions();
  m_options = GraphOfConvexSetsOptions();  // todo: enable modifying these options

  // fill in subscription variables with defaults if they weren't set in config
  // leave these variables empty in constructor so that we don't register for
  // variables we don't need
  if (m_path_request_var.empty()) m_path_request_var = "PLAN_PATH_REQUESTED";
  if (m_wpt_complete_var.empty()) m_wpt_complete_var = "WAYPOINTS_COMPLETE";
  if (m_clear_iris_var.empty()) m_clear_iris_var = "CLEAR_IRIS";
  if (m_iris_region_var.empty()) m_iris_region_var = "IRIS_REGION";
  if (m_iris_complete_var.empty()) m_iris_complete_var = "IRIS_COMPLETE";

  registerVariables();
  return(true);
}


bool GCS2D::readRegionsFromFile()
{
  // todo: support reading iris regions from a file
  return (false);
}


void GCS2D::checkConfigAssertions()
{
  // ordr and continuity must be at least 1 and 0 respectively
  if (m_bezier_order < 1) {
    reportConfigWarning("Cannot have bezier order < 1, setting to 1.");
    m_bezier_order = 1;
  }

  if (m_bezier_continuity_req < 0) {
    reportConfigWarning("Cannot have bezier continuity < 0, setting to 0.");
    m_bezier_continuity_req = 1;
  }

  // can only differentiate a bezier curve (order - 1) times, so limit continuity constraints
  if (m_bezier_continuity_req > m_bezier_order - 1) {
    reportConfigWarning("Path derivative ontinuity constraints exceed Bezier order; "
                        "limiting constraints to {bezier_order} - 1.");
    m_bezier_continuity_req = m_bezier_order - 1;
  }

  // make sure derivative regularization, if used, applies to valid derivatives
  if (m_derivative_regularization_weight > 0) {
    if ((m_derivative_regularization_order < 2) ||
        (m_derivative_regularization_order > m_bezier_order)) {
          reportConfigWarning("Derivative regularization is not of order >= 2 and <= "
                              "Bezier order. Disabling derivative regularization.");
          m_derivative_regularization_weight = 0;
          m_derivative_regularization_order = 0;
        }
  }
}


//---------------------------------------------------------

std::string GCS2D::getPathStats()
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


bool GCS2D::postPath()
{
  Notify(m_prefix + m_path_found_var, "true");
  Notify(m_prefix + m_path_stats_var, getPathStats());
  m_next_path_idx = 0;
  postFlags(m_traverse_flags);
  return (true);
}


//---------------------------------------------------------

std::string GCS2D::printPlannerMode()
{
  switch (m_mode)
  {
  case PlannerMode::IDLE:
    return ("IDLE");
  case PlannerMode::REQUEST_PENDING:
    return ("REQUEST_PENDING");
  case PlannerMode::IRIS_IN_PROGRESS:
    return ("IRIS_IN_PROGRESS");
  case PlannerMode::GCS_IN_PROGRESS:
    return ("GCS_IN_PROGRESS");
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


std::string GCS2D::printGCSStep()
{
  switch (m_gcs_step)
  {
  case (GCSStep::BUILD_GRAPH):
    return ("BUILD_GRAPH");
  case (GCSStep::PREPROCESS_GRAPH):
    return ("PREPROCESS_GRAPH");
  case (GCSStep::POPULATE_MODEL):
    return ("POPULATE_MODEL");
  case (GCSStep::START_MOSEK):
    return ("START_MOSEK");
  case (GCSStep::MOSEK_RUNNING):
    return ("MOSEK_RUNNING");
  case (GCSStep::CONVEX_ROUNDING):
    return ("CONVEX_ROUNDING");
  case (GCSStep::FAILED):
    return ("FAILED");
  default:
    return ("UNKNOWN");
  }
}


void GCS2D::postFlags(const std::vector<VarDataPair>& flags)
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
// Procedure: registerVariables()

void GCS2D::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("WPT_INDEX", 0);
  if (!m_path_request_var.empty()) Register(m_path_request_var, 0);
  if (!m_wpt_complete_var.empty()) Register(m_wpt_complete_var, 0);
  if (!m_clear_iris_var.empty()) Register(m_clear_iris_var, 0);
  if (!m_iris_region_var.empty()) Register(m_iris_region_var, 0);
  if (!m_iris_complete_var.empty()) Register(m_iris_complete_var, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool GCS2D::buildReport()
{
  using std::endl;
  std::string header{"================================"};
  m_msgs << header << endl;
  m_msgs << "Subscriptions:" << endl;
  m_msgs << "  path_request_var: " << m_path_request_var << endl;
  m_msgs << "  wpt_complete_var: " << m_wpt_complete_var << endl;

  m_msgs << header << endl;
  m_msgs << "Flags:" << endl;
  m_msgs << "  Initial Plan Flags:" << endl;
  for (VarDataPair pair : m_init_plan_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  Traverse Flags:" << endl;
  for (VarDataPair pair : m_traverse_flags)
    m_msgs << "    " << pair.getPrintable() << endl;
  m_msgs << "  End Flags:" << endl;
  for (VarDataPair pair : m_end_flags)
    m_msgs << "    " << pair.getPrintable() << endl;

  m_msgs << header << endl;
  m_msgs << "Planner Mode: " << printPlannerMode() << endl;
  m_msgs << "Safe Regions: " << std::to_string(m_safe_regions.size()) << endl;
  std::string start_spec{m_start_point.valid() ? m_start_point.get_spec() : "UNSET"};
  std::string goal_spec{m_goal_point.valid() ? m_goal_point.get_spec() : "UNSET"};
  m_msgs << "Start Point: " << start_spec << endl;
  m_msgs << "Goal Point: " << goal_spec << endl;
  if (!m_prefix.empty())
    m_msgs << "Publication Prefix: " << m_prefix << endl;

  m_msgs << header << endl;
  m_msgs << "GCS Config:" << endl;
  m_msgs << "  Bezier Order: " << intToString(m_bezier_order) << endl;
  m_msgs << "  Bezier Continuity Requirement: " << intToString(m_bezier_continuity_req) << endl;
  m_msgs << "  Path Length Weight: " << doubleToStringX(m_path_length_weight, 3) << endl;
  m_msgs << "  Derivative Regularization Weight: " <<
    doubleToStringX(m_derivative_regularization_weight, 3) << endl;
  m_msgs << "  Derivative Regularization Order: " <<
    intToString(m_derivative_regularization_order) << endl;
  // todo: add gcs optimization options to report

  m_msgs << header << endl;
  m_msgs << "IRIS Config:" << endl;
  if (!m_iris_file.empty()) {
    m_msgs << "  Safe Regions File: " << m_iris_file << endl;
    m_msgs << "  iris_clear_var: " << m_clear_iris_var << endl;
  } else {
    m_msgs << "  Rerun IRIS on new path request: " << boolToString(m_run_iris_on_new_path) << endl;
    m_msgs << "  iris_run_var: " << m_run_iris_var << endl;
    m_msgs << "  iris_clear_var: " << m_clear_iris_var << endl;
    m_msgs << "  iris_region_var: " << m_iris_region_var << endl;
    m_msgs << "  iris_complete_var: " << m_iris_complete_var << endl;
  }

  m_msgs << header << endl;
  if ((m_mode == PlannerMode::IRIS_IN_PROGRESS) || (m_mode == PlannerMode::GCS_IN_PROGRESS)) {
    m_msgs << "PLANNING IN PROGRESS" << endl;
    m_msgs << "GCS Step: " << printGCSStep() << endl;
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
