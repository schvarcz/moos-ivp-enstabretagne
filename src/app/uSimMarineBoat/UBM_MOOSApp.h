/*****************************************************************/
/*    NAME: Dominique Monnet                                     */
/*    ORGN: ENSTA-Bretagne                                       */
/*    FILE: UBM_Model.h                                          */
/*    DATE: Sep 27, 2016                                         */
/*                                                               */
/*****************************************************************/

#ifndef UBM_MOOSAPP_HEADER
#define UBM_MOOSAPP_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "UBM_Model.h"

class UBM_MOOSApp : public AppCastingMOOSApp
{
public:
  UBM_MOOSApp();
  virtual ~UBM_MOOSApp() {}

 public: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool OnStartUp();
  bool Iterate();
  bool OnConnectToServer();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();
  void postNodeRecordUpdate(std::string, const NodeRecord&);
  bool handleThrustMapping(std::string);
  void cacheStartingInfo();
  
  std::string handleConfigDeprecations(std::string);

protected:
  std::string  m_sim_prefix;
  UBM_Model    m_model;
  unsigned int m_reset_count;

  CMOOSGeodesy m_geodesy;
  bool         m_geo_ok;
  bool         m_thrust_mode_reverse;
  bool         m_thrust_mode_differential;

  // A cache of starting info to facilitate generation of reports.
  std::string m_start_nav_x;
  std::string m_start_nav_y;
  std::string m_start_nav_hdg;
  std::string m_start_nav_spd;
  std::string m_start_nav_dep;
  std::string m_start_nav_alt;
  std::string m_start_nav_lat;
  std::string m_start_nav_lon;
  std::string m_start_buoyrate;
  std::string m_start_drift_x;
  std::string m_start_drift_y;
  std::string m_start_drift_mag;
  std::string m_start_drift_ang;
  std::string m_start_rotate_spd;

  std::set<std::string> m_srcs_buoyrate;
  std::set<std::string> m_srcs_drift;

  bool   buoyancy_requested;
  bool   trim_requested;
  double buoyancy_request_time;
  double trim_request_time;
  double buoyancy_delay;
  double max_trim_delay;
  double last_report;
  double report_interval;
  double pitch_tolerance;
  
  bool   m_obstacle_hit;
};

#endif
