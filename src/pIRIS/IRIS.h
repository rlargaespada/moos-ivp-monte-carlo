/************************************************************/
/*    NAME: Raul Largaespada                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IRIS.h                                          */
/*    DATE: June 14th, 2023                                 */
/************************************************************/

#ifndef IRIS_HEADER
#define IRIS_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class IRIS : public AppCastingMOOSApp
{
 public:
  IRIS();
  ~IRIS();

 protected:  // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected:  // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();

 private:  // Configuration variables
 private:  // State variables
};

#endif
