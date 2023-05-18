/****************************************************************/
/*   NAME: Raul Largaespada                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: NullPlanner_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "NullPlanner_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pNullPlanner application plans a path from a given start point");
  blk("  to a given goal point by returning a waypoint to the start followed");
  blk("  by a waypoint to the goal, with configurable intermediate points");
  blk("  added as waypoints in between.                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pNullPlanner file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pNullPlanner with the given process name         ");
  blk("      rather than pNullPlanner.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pNullPlanner.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pNullPlanner Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pNullPlanner                                    ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  path_request_var = PLAN_PATH_REQUESTED  // default is PLAN_PATH_REQUESTED");
  blk("  obs_alert_var = OBSTACLE_ALERT  // default is OBSTACLE_ALERT  ");
  blk("  wpt_complete_var = WAYPOINTS_COMPLETE  // default is WAYPOINTS_COMPLETE");
  blk("                                                                ");
  blk("  prefix = alpha  // default is no prefix                       ");
  blk("  intermediate_pts = 0,0:200,-150:100,100                       ");
  blk("  init_flag = DEPLOY=true  // example, any number of flags are supported");
  blk("                           // through separate init_flag config lines");
  blk("  traverse_flag = TRAVERSE=true  // example, any number of flags are supported");
  blk("                                 // through separate traverse_flag config lines");
  blk("  end_flag = DEPLOY=false  // example, any number of flags are supported");
  blk("                           // through separate end_flag config lines");
  blk("  post_visuals = true  // default is true                       ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pNullPlanner INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT                                                   ");
  blk("  OBM_RESOLVED = obs_key                                        ");
  blk("  WPT_INDEX = 0                                                 ");
  blk("  <path_request_var> = start=100,220;goal=210,30  // set in config");
  blk("  <obs_alert_var> = name=d#poly  // set in config               ");
  blk("                    poly={32,-100:38,-98:40,-100:32,-104}       ");
  blk("                    label=obs_key                               ");
  blk("  <wpt_complete_var>  // set in config                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  <prefix>_PATH_FOUND = true  // prefix set in config           ");
  blk("  <prefix>_PATH_COMPLETE = true  // prefix set in config        ");
  blk("  <prefix>_PATH_FAILED = false  // prefix set in config         ");
  blk("  <prefix>_PATH_STATS = algorithm=none,  // prefix set in config");
  blk("                        planning_time=1,                        ");
  blk("                        path_len_to_go=100,                     ");
  blk("                        path_len_traversed=200,                 ");
  blk("  <init_flag>  // set in config                                 ");
  blk("  <traverse_flag>  // set in config                             ");
  blk("  <end_flag>  // set in config                                  ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pNullPlanner", "gpl");
  exit(0);
}

