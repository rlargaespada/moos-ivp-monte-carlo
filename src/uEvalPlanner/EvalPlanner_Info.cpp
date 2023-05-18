/****************************************************************/
/*   NAME: Raul Largaespada                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: EvalPlanner_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "EvalPlanner_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"


//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The uEvalPlanner app tracks the performance of a path planning");
  blk("  algorithm running on the vehicle by directing the vehicle to  ");
  blk("  plan a path from a start point to a goal point and tracking   ");
  blk("  performance metrics as the vehicle follows this path.         ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uEvalPlanner file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                     ");
  blk("      Launch uEvalPlanner with the given process name         ");
  blk("      rather than uEvalPlanner.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of uEvalPlanner.        ");
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
  blu("uEvalPlanner Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uEvalPlanner                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  vehicle_name = alpha  // required, must match name of MOOS community on vehicle");
  blk("  timeout = 300  // default is 300 seconds                      ");
  blk("  start_pos = x=100,y=220  // any XYPoint spec format is valid  ");
  blk("  goal_pos = x=210,y=30  // any XYPoint spec format is valid    ");
  blk("  heading_on_reset = 0  // \"relative\" is also valid, for the relative");
  blk("                        // angle between the start and goal points");
  blk("                                                                ");
  blk("  path_request_var = PLAN_PATH_REQUESTED  // default is PLAN_PATH_REQUESTED");
  blk("  path_complete_var = PATH_COMPLETE  // default is PATH_COMPLETE");
  blk("  path_stats_var = PATH_STATS  // default is PATH_STATS         ");
  blk("  path_failed_var = PATH_FAILED  // default is PATH_FAILED      ");
  blk("                                                                ");
  blk("  num_trials = 10  // default is 10                             ");
  blk("  obs_reset_var = UFOS_RESET  // default is UFOS_RESET. Any number");
  blk("                              // of variables is supported. Set to");
  blk("                              // \"none\" to not reset any obstacles.");
  blk("  deviation_limit = 5  // default is 5 meters                   ");
  blk("                                                                ");
  blk("  trial_flag = DEPLOY=true  // example, any number of flags are supported");
  blk("                            // through separate trial_flag config lines");
  blk("  end_flag = DEPLOY=false  // example, any number of flags are supported");
  blk("                           // through separate end_flag config lines");
  blk("                                                                ");
  blk("  exportfile = uEvalPlanner_Metrics  // default is uEvalPlanner_Metrics");
  blk("  use_timestamp = true  // default is true                      ");
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
  blu("uEvalPlanner INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  red("  <vehicle_name> set in configuration parameter \"vehicle_name\"  ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  RESET_SIM_REQUESTED = {all, <vehicle_name>}                   ");
  blk("  END_SIM_REQUESTED = {all, <vehicle_name>}                     ");
  blk("  RESET_TRIAL_REQUESTED = {all, <vehicle_name>}                 ");
  blk("  SKIP_TRIAL_REQUESTED = {all, <vehicle_name>}                  ");
  blk("                                                                ");
  blk("  <path_complete_var> = true  // set in config, default is PATH_COMPLETE");
  blk("  <path_stats_var>  // set in config, default is PATH_STATS     ");
  blk("  <path_failed_var> = true  // set in config, default is PATH_FAILED");
  blk("                                                                ");
  blk("  UEP_START_POS = vname=<vehicle_name>,x=100,y=220              ");
  blk("  UEP_GOAL_POS = vname=<vehicle_name>,x=210,y=30                ");
  blk("                                                                ");
  blk("  KNOWN_OBSTACLE_CLEAR                                          ");
  blk("  NODE_REPORT                                                   ");
  blk("                                                                ");
  blk("  ENCOUNTER_ALERT = vname=<vehicle_name>, dist=20               ");
  blk("  NEAR_MISS_ALERT = vname=<vehicle_name>, dist=10               ");
  blk("  COLLISION_ALERT = vname=<vehicle_name>, dist=5                ");
  blk("                                                                ");
  blk("  UPC_ODOMETRY_REPORT = vname=<vehicle_name>,total_dist=4205.4,trip_dist=1105.2");
  blk("  WPT_ADVANCED = prev=$[PX],$[PY];next=$[NX],$[NY]              ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  <obs_reset_var> = now  // set in config                       ");
  blk("  USM_RESET_<vehicle_name> = x=100,y=220,speed=0,heading=10,depth=10");
  blk("  UPC_TRIP_RESET = <vehicle_name                                ");
  blk("  <path_request_var> = start=100,220;goal=210,30  // set in config");
  blk("  <trial_flag> // set in config                                 ");
  blk("  <end_flag> // set in config                                   ");
  blk("  METRICS_EXPORTED = uEvalPlanner_Metrics.csv                   ");
  blk("  TRIALS_COMPLETED = 3                                          ");
  blk("  TRIAL_STATS                                                   ");
  blk("  VIEW_MARKER                                                   ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uEvalPlanner", "gpl");
  exit(0);
}

