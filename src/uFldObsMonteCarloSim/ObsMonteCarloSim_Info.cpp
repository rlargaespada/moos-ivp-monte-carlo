/****************************************************************/
/*   NAME: Raul Largaespada                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: ObsMonteCarloSim_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ObsMonteCarloSim_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"


//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The uFldObsMonteCarloSim app will simulate the posting of     ");
  blk("  obstacles loaded from a text file, to be shared to all        ");
  blk("  vehicles in the uField environment. It features additional    ");
  blk("  configuration parameters and functionality compared to the    ");
  blk("  original uFldObstacleSim app.                                 ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uFldObsMonteCarloSim file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                      ");
  blk("      Launch uFldObsMonteCarloSim with the given process name         ");
  blk("      rather than uFldObsMonteCarloSim.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of uFldObsMonteCarloSim.        ");
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
  blu("uFldObsMonteCarloSim Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uFldObsMonteCarloSim                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  obstacle_file     = obstacles.txt                             ");
  blk("  obstacle_file_var = NEW_OBSTACLE_FILE  (default is NEW_OBSTACLE_FILE)");
  blk("  label_prefix      = known   (default is no prefix)            ");
  blk("                                                                ");
  blk("  poly_vert_color  = color    (default is gray50)               ");
  blk("  poly_edge_color  = color    (default is gray50)               ");
  blk("  poly_fill_color  = color    (default is white)                ");
  blk("  poly_label_color = color    (default is invisible)            ");
  blk("                                                                ");
  blk("  poly_vert_size    = 1       (default is 1)                    ");
  blk("  poly_edge_size    = 1       (default is 1)                    ");
  blk("  poly_transparency = 0.15    (default is 0.15)                 ");
  blk("                                                                ");
  blk("  draw_region       = true    (default is true)                 ");
  blk("  region_edge_color = color   (default is gray50)               ");
  blk("  region_vert_color = color   (default is white)                ");
  blk("                                                                ");
  blk("  post_points      = true     (default is false)                ");
  blk("  rate_points      = 5        (default is 5)                    ");
  blk("  point_size       = 5        (default is 2)                    ");
  blk("                                                                ");
  blk("  min_duration     = 10       (default is -1)                   ");
  blk("  max_duration     = 15       (default is -1)                   ");
  blk("  refresh_interval = 8        (default is -1)                   ");
  blk("                                                                ");
  blk("  reset_interval   = -1       (default is -1)                   ");
  blk("  reset_range      = 10       (default is 10)                   ");
  blk("  reset_var        = UFOS_RESET  (default is UFOS_RESET, set to \"none\"");
  blk("                                  to disable resets from variable postings)");
  blk("                                                                ");
  blk("  reuse_ids        = true     (default is true)                 ");
  blk("  sensor_range     = 50       (default is 50)                   ");
  blk("                                                                ");
  blk("  app_logging  = true  // {true or file} By default disabled    ");
  blk("                                                                ");
  blk("  post_visuals = true  // {true or false} By default true       ");
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
  blu("uFldObsMonteCarloSim INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  PMV_CONNECT      = true                                       ");
  blk("  OBM_CONNECT      = true                                       ");
  blk("  VEHICLE_CONNECT  = true                                       ");
  blk("  <reset_var>      = now  // set in config, default is UFOS_RESET");
  blk("  UFOS_POINT_SIZE  = <double>  // \"smaller\" and \"bigger\" are also acceptable values");
  blk("  <obstacle_file_var> = obstacles.txt  // set in config, default is NEW_OBSTACLE_FILE");

  blk("  NODE_REPORT                                                   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_POLYGON= <polygon_spec>                                  ");
  blk("  VIEW_POINT = <point_spec>                                     ");
  blk("  KNOWN_OBSTACLE = <polygon_spec>                               ");
  blk("  KNOWN_OBSTACLE_CLEAR = <app_name>                             ");
  blk("  GIVEN_OBSTACLE = <polygon_spec>                               ");
  blk("  TRACKED_FEATURE_<VNAME> = <point_spec>  // vehicle names sourced from node reports");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uFldObsMonteCarloSim", "gpl");
  exit(0);
}

