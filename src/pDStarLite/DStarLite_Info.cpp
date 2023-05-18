/****************************************************************/
/*   NAME: Raul Largaespada                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: DStarLite_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "DStarLite_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"


//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pDStarLite application plans a path from a given start point");
  blk("  to a given goal point using the D* Lite algorithm.            ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pDStarLite file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                      ");
  blk("      Launch pDStarLite with the given process name         ");
  blk("      rather than pDStarLite.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pDStarLite.        ");
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
  blu("pDStarLite Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pDStarLite                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  path_request_var = PLAN_PATH_REQUESTED  // default is PLAN_PATH_REQUESTED");
  blk("  obs_alert_var = OBSTACLE_ALERT  // default is OBSTACLE_ALERT  ");
  blk("  wpt_complete_var = WAYPOINTS_COMPLETE  // default is WAYPOINTS_COMPLETE");
  blk("                                                                ");
  blk("  prefix = alpha  // default is no prefix                       ");
  blk("  init_flag = DEPLOY=true  // example, any number of flags are supported");
  blk("                           // through separate init_flag config lines");
  blk("  traverse_flag = TRAVERSE=true  // example, any number of flags are supported");
  blk("                                 // through separate traverse_flag config lines");
  blk("  replan_flag = TRAVERSE=false  // example, any number of flags are supported");
  blk("                                // through separate replan_flag config lines");
  blk("  end_flag = DEPLOY=false  // example, any number of flags are supported");
  blk("                           // through separate end_flag config lines");
  blk("  post_visuals = true  // default is true                       ");
  blk("                                                                ");
  blk("  grid_bounds = {-150, 50:-150,-250:250,-250:250,50}  // mandatory");
  blk("  grid_cell_size = 5  // default is 5 m                         ");
  blk("  max_planning_iters = 200  // default is 200                   ");
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
  blu("pDStarLite INTERFACE                                    ");
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
  blk("  <replan_flag>  // set in config                               ");
  blk("  <end_flag>  // set in config                                  ");
  blk("  VIEW_GRID                                                     ");
  blk("  VIEW_GRID_DELTA                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pDStarLite", "gpl");
  exit(0);
}

