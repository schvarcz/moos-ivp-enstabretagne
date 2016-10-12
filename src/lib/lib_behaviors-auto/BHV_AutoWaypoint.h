#ifndef BHV_AUTOWAYPOINT_HEADER
#define BHV_AUTOWAYPOINT_HEADER

#include <string>
#include "IvPBehavior.h"
#include "WaypointEngine.h"
#include "XYPoint.h"

class BHV_AutoWaypoint : public IvPBehavior {
public:
  BHV_AutoWaypoint(IvPDomain);
  ~BHV_AutoWaypoint() {}
  
  bool           setParam(std::string, std::string);
  IvPFunction*   onRunState();
  void           onIdleState();
  void           onIdleToRunState();
  BehaviorReport onRunState(std::string);
  void           onRunToIdleState();
  void           onSetParamComplete();
  void           onCompleteState() {postErasables();}

protected:
  bool         updateInfoIn();
  bool         setNextWaypoint();
  IvPFunction* buildOF(std::string);

  void         postStatusReport();
  void         postViewableSegList();
  void         postErasableSegList();
  void         postErasables();
  void         postCycleFlags();
  void         postWptFlags(double x, double y);
  void         handleVisualHint(std::string);
  void         updateOdoDistance();
  void         markOdoLeg();

protected: 
  WaypointEngine m_waypoint_engine;

protected: // configuration parameters
  double      m_cruise_speed;
  bool        m_lead_to_start;
  double      m_lead_distance;
  double      m_lead_damper;
  std::string m_efficiency_measure;
  std::string m_ipf_type;
  bool disp_error; // post distance to line error
  double total_error; // total error to track line

  // Configurable names of MOOS variables for reports
  std::string m_var_report;
  std::string m_var_index;
  std::string m_var_cyindex;
  std::string m_var_suffix;
  std::string m_var_dist_to_prev;
  std::string m_var_dist_to_next;

  // Var-Data flags for posting when behavior finishes cycle
  std::vector<VarDataPair> m_cycle_flags;
  std::vector<VarDataPair> m_wpt_flags;

  // Visual hints affecting properties of polygons/points
  std::string m_hint_vertex_color;
  std::string m_hint_edge_color;
  std::string m_hint_nextpt_color;
  std::string m_hint_nextpt_lcolor;
  double      m_hint_vertex_size;
  double      m_hint_edge_size;
  double      m_hint_nextpt_vertex_size;

protected: // intermediate or object global variables.
  double    m_osh;  // Ownship heading
  double    m_osv;  // Ownship velocity
  double    m_osx;  // Ownship x position
  double    m_osy;  // Ownship y position
  
  // Time/Distance efficiency state information
  bool      m_odo_set_flag;
  bool      m_odo_leg_disq;
  double    m_odo_settime;
  double    m_odo_setx;
  double    m_odo_sety;
  bool      m_odo_virgin;

  double    m_dist_leg_odo;

  double    m_dist_total_odo;
  double    m_dist_total_linear;
  double    m_time_total_odo;
  double    m_time_total_linear;

  double    m_course_pct;
  double    m_speed_pct;

  double    m_osx_prev;
  double    m_osy_prev;

  XYPoint   m_nextpt;
  XYPoint   m_trackpt;
  XYPoint   m_markpt;

  XYPoint   m_prevpt;

  bool      m_greedy_tour_pending;
};

#ifdef WIN32
  // Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_AutoWaypoint(domain);}
}
#endif