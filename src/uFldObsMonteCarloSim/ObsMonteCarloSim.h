/************************************************************/
/*    NAME: Raul Largaespada                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ObsMonteCarloSim.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ObsMonteCarloSim_HEADER
#define ObsMonteCarloSim_HEADER

#include <map>
#include <random>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "VarDataPair.h"
#include "XYPolygon.h"
#include "XYVector.h"


class ObsMonteCarloSim : public AppCastingMOOSApp
{
 public:
  ObsMonteCarloSim();
  ~ObsMonteCarloSim();

 protected:  // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:  // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();

  bool handleConfigMinDuration(std::string);
  bool handleConfigMaxDuration(std::string);
  bool handleConfigObstacleFile(std::string filename);
  void handleConfigObstacleDurations();
  bool handleConfigDriftVector(std::string vector_str);

  bool handleMailNodeReport(std::string);
  bool handleMailPointSize(std::string);

  void postObstaclesRefresh();
  void postObstaclesErase();
  void postPoints();

  void updateVRanges();
  void updateObstaclesField();
  bool generateObstacle(std::vector<XYPolygon>* obs_vec, unsigned int tries);
  void updateObstaclesFromFile();
  void updateObstaclesRefresh();

 private:  // Configuration variables
  // Parameters if we're creating obstacles as we go
  XYPolygon   m_poly_region;
  double      m_min_range;
  double      m_min_poly_size;
  double      m_max_poly_size;
  bool        m_reuse_ids;
  std::string m_label_prefix;

  // Parameters for reading obstacles from a file
  std::string m_obstacle_file_var;

  // Visual params for rendering obstacles
  bool   m_post_visuals;
  std::string m_poly_fill_color;
  std::string m_poly_edge_color;
  std::string m_poly_vert_color;
  std::string m_poly_label_color;

  double      m_poly_edge_size;
  double      m_poly_vert_size;
  double      m_poly_transparency;

  // Visual params for rendering region
  bool        m_draw_region;
  std::string m_region_edge_color;
  std::string m_region_vert_color;

  // Pseudo LIDAR generation mode
  bool    m_post_points;
  double  m_rate_points;
  double  m_point_size;
  double m_sensor_range;  // range of the sensor for generating point updates

  // Params for random durations
  double  m_min_duration;
  double  m_max_duration;
  double  m_obs_refresh_interval;

  // Params for resetting the obs field
  double       m_reset_interval;
  double       m_reset_range;
  std::string  m_reset_var;

  // Params for obstacle drift
  int m_num_drifting_obs;

 private:  // State variables
  // Core list of obtacles
  std::vector<XYPolygon> m_obstacles;
  std::vector<double>    m_durations;

  // Maps keyed on vnames.
  std::map<std::string, NodeRecord>  m_map_vrecords;
  std::map<std::string, double>      m_map_vrange;

  // Maps keyed on obstacle key/label
  std::map<std::string, unsigned int> m_map_pts_published;
  std::map<std::string, unsigned int> m_map_giv_published;

  double       m_reset_tstamp;
  bool         m_reset_request;
  bool         m_reset_pending;
  std::string  m_new_obstacle_file;
  bool         m_newly_exited;
  bool         m_region_entered;
  unsigned int m_reset_total;

  double  m_min_vrange_to_region;

  bool    m_obs_refresh_needed;
  double  m_obs_refresh_tstamp;

  unsigned int m_obstacles_made;
  unsigned int m_obstacles_posted;

  // random number generator for generating obstacles
  std::default_random_engine m_generator;
  std::uniform_real_distribution<double> m_obs_scaling{0, 1};

  // obstacle drift state
  XYVector m_drift_vector;
  double m_rotate_speed;
};

#endif
