/*****************************************************************/
/*    NAME: Dominique Monnet                                     */
/*    ORGN: ENSTA-Bretagne                                       */
/*    FILE: UBM_Model.cpp                                        */
/*    DATE: Sep 27, 2016                                         */
/*                                                               */
/*****************************************************************/

#include <iostream>
#include <cmath> 
#include <cstdlib>
#include "UBM_Model.h"
#include "SimEngineBoat.h"
#include "MBUtils.h"
#include "AngleUtils.h"

using namespace std;

//------------------------------------------------------------------------
// Constructor

UBM_Model::UBM_Model()
{
  // Initalize the configuration variables
  m_dual_state           = false;
  m_paused               = false;
  m_turn_rate            = 70;
  m_max_acceleration     = 0;
  m_max_deceleration     = 0.5;
  m_buoyancy_rate        = 0.025;  // positively buoyant
  m_max_depth_rate       = 0.5;    // meters per second
  m_max_depth_rate_speed = 2.0;    // meters per second

  m_max_rudder_degs_per_sec = 0;

  m_thrust_map.setThrustFactor(20);

  // Initalize the state variables
  m_rudder       = 0;
  m_rudder_prev  = 0;
  m_rudder_tstamp = 0;

  m_thrust       = 0;
  m_elevator     = 0;
  m_drift_x      = 0;
  m_drift_y      = 0; 
  m_rotate_speed = 0;
  m_drift_fresh  = true;
  m_water_depth  = 0;    // zero means nothing known, no altitude reported

  m_thrust_mode  = "normal";  // vs. "differential"
  m_thrust_lft   = 0;
  m_thrust_rgt   = 0;

  m_thrust_mode_reverse = false;
}

//------------------------------------------------------------------------
// Procedure: resetTime()

void UBM_Model::resetTime(double g_curr_time)
{
  m_record.setTimeStamp(g_curr_time);
  m_record_gt.setTimeStamp(g_curr_time);
}
//------------------------------------------------------------------------
// Procedure: setParam

bool UBM_Model::setParam(string param, double value)
{
  param = stripBlankEnds(tolower(param));
  if(param == "start_x") {
    m_record.setX(value);
    m_record_gt.setX(value);
  }
  else if(param == "start_y") {
    m_record.setY(value);
    m_record_gt.setY(value);
  }
  else if(param == "start_heading") {
    m_record.setHeading(value);
    m_record_gt.setHeading(value);
  }
  else if(param == "start_speed") {
    m_record.setSpeed(value);
    m_record_gt.setSpeed(value);
  }
  else if(param == "start_depth") {
    m_record.setDepth(value);
    m_record_gt.setDepth(value);
    if(value < 0) {
      m_record.setDepth(0);
      m_record_gt.setDepth(0);
      return(false);
    }
  }
  else if(param == "buoyancy_rate")
    m_buoyancy_rate = value;
  else if(param == "turn_rate")
    m_turn_rate = vclip(value, 0, 100);
  else if(param == "drift_x")
    m_drift_x = value;
  else if(param == "drift_y")
    m_drift_y = value;
  else if(param == "rotate_speed")
    m_rotate_speed = value;
  else if(param == "max_acceleration") {
    m_max_acceleration = value;
    if(m_max_acceleration < 0) {
      m_max_acceleration = 0;
      return(false);
    }
  }
  else if(param == "max_deceleration") {
    m_max_deceleration = value;
    if(m_max_deceleration < 0) {
      m_max_deceleration = 0;
      return(false);
    }
  }
  else if(param == "max_depth_rate")
    m_max_depth_rate = value;
  else if(param == "max_depth_rate_speed")
    m_max_depth_rate_speed = value;
  else if(param == "water_depth") {
    if(value >= 0)
      m_water_depth = value;
    else
      return(false);
  }
  else if(param == "friction_coef")
      m_friction_coef = value;
  else
    return(false);
  return(true);
}

//------------------------------------------------------------------------
// Procedure: setRudder()

void UBM_Model::setRudder(double desired_rudder, double tstamp)
{
  // Part 0: Copy the current rudder value to "previous" before overwriting
  m_rudder_prev = m_rudder;


  // Part 1: Calculate the maximum change in rudder
  double max_rudder_change = 100;
  if(m_max_rudder_degs_per_sec > 0) {
    double delta_time = tstamp - m_rudder_tstamp;
    max_rudder_change = (delta_time * m_max_rudder_degs_per_sec);
  }

  // Part 2: Handle the change requested
  double change = desired_rudder - m_rudder_prev;
  if(change > 0) { 
    if(change > max_rudder_change)
      change = max_rudder_change;
    m_rudder += change;
  }
  else {
    if(-change > max_rudder_change)
      change = -max_rudder_change;
    m_rudder += change;
  }

  m_thrust_mode = "normal";
  m_rudder_tstamp = tstamp;
}


//------------------------------------------------------------------------
// Procedure: propagate

bool UBM_Model::propagate(double g_curr_time)
{
  if(m_paused) {
    cout << "Simulator PAUSED..................." << endl;
    return(true);
  }
    
  // Calculate actual current time considering time spent paused.
  double a_curr_time = g_curr_time - m_pause_timer.get_wall_time();
  double delta_time  = a_curr_time - m_record.getTimeStamp();

  if(m_dual_state) {
    propagateNodeRecord(m_record, delta_time, false);
    propagateNodeRecord(m_record_gt, delta_time, true);
  }
  else
    propagateNodeRecord(m_record, delta_time, true);
    

  return(true);
}

//--------------------------------------------------------------------
// Procedure: setDriftVector(string, bool)

bool UBM_Model::setDriftVector(string str, bool add_new_drift)
{
  string left  = biteStringX(str, ',');
  string right = str;

  if(!isNumber(left) || !isNumber(right))
    return(false);
  
  double ang  = angle360(atof(left.c_str()));
  double mag  = atof(right.c_str());
  double rads = headingToRadians(ang);

  double xmps = cos(rads) * mag;
  double ymps = sin(rads) * mag;

  if(add_new_drift) {
    m_drift_x += xmps;
    m_drift_y += ymps;
  }
  else {
    m_drift_x = xmps;
    m_drift_y = ymps;
  }

  m_drift_fresh = true;
  return(true);
}

//--------------------------------------------------------------------
// Procedure: magDriftVector(double)
//   Purpose: Grow the existing drift vector by the specified percent.
//            Negative values allowed, but each will flip the direction
//            of the vector.

void UBM_Model::magDriftVector(double pct)
{
  double ang = relAng(0, 0, m_drift_x, m_drift_y);
  double mag = hypot(m_drift_x, m_drift_y);

  double new_mag = mag * pct;
  double rads = headingToRadians(ang);

  double xmps = cos(rads) * new_mag;
  double ymps = sin(rads) * new_mag;

  m_drift_x = xmps;
  m_drift_y = ymps;

  m_drift_fresh = true;
}

//------------------------------------------------------------------------
// Procedure: setMaxRudderDegreesPerSec()

bool UBM_Model::setMaxRudderDegreesPerSec(double v)
{
  if(v < 0)
    return(false);

  m_max_rudder_degs_per_sec = v;
  return(true);
}

//------------------------------------------------------------------------
// Procedure: setPaused

void UBM_Model::setPaused(bool g_paused)
{
  if(m_paused == g_paused)
    return;
  else
    m_paused = g_paused;
 
  if(m_paused)
    m_pause_timer.start();
  else
    m_pause_timer.stop();  
}

//------------------------------------------------------------------------
// Procedure: setThrustFactor

void UBM_Model::setThrustFactor(double value)
{
  m_thrust_map.setThrustFactor(value);
}

//------------------------------------------------------------------------
// Procedure: setThrustReflect

void UBM_Model::setThrustReflect(bool value)
{
  m_thrust_map.setReflect(value);
}

//------------------------------------------------------------------------
// Procedure: setThrustLeft

void UBM_Model::setThrustLeft(double val)
{
  if(val < -100)
    val = -100;
  else if(val > 100)
    val = 100;

    m_thrust_mode = "differential";

  if(m_thrust_mode_reverse == false)  // The normal mode
    m_thrust_lft  = val;
  else
    m_thrust_rgt  = -val;
}

//------------------------------------------------------------------------
// Procedure: setThrustRight

void UBM_Model::setThrustRight(double val)
{
  if(val < -100)
    val = -100;
  else if(val > 100)
    val = 100;

  m_thrust_mode = "differential";
  if(m_thrust_mode_reverse == false)  // The normal mode
    m_thrust_rgt  = val;
  else
    m_thrust_lft  = -val;
}

//--------------------------------------------------------------------- 
// Procedure: addThrustMapping

bool UBM_Model::addThrustMapping(double thrust, double speed)
{
  bool result = m_thrust_map.addPair(thrust, speed);
  return(result);
}

//---------------------------------------------------------------------
// Procedure: getDriftSummary()

string UBM_Model::getDriftSummary()
{
  // Revert to c^2 = a^2 + b^2 
  double c_squared = (m_drift_x * m_drift_x) + (m_drift_y * m_drift_y);
  double magnitude = sqrt(c_squared);
  double angle = relAng(0, 0, m_drift_x, m_drift_y);

  string val = "ang=";
  val += doubleToStringX(angle,2);
  val += ", mag=";
  val += doubleToStringX(magnitude,2);
  val += ", xmag=";
  val += doubleToStringX(m_drift_x,3);
  val += ", ymag=";
  val += doubleToStringX(m_drift_y,3);
  return(val);
}


//------------------------------------------------------------------------
// Procedure: usingThrustFactor

bool UBM_Model::usingThrustFactor() const
{
  return(m_thrust_map.usingThrustFactor());
}

//------------------------------------------------------------------------
// Procedure: getThrustFactor

double UBM_Model::getThrustFactor() const
{
  return(m_thrust_map.getThrustFactor());
}


//------------------------------------------------------------------------
// Procedure: initPosition
//
//  "x=20, y=-35, speed=2.2, heading=180, depth=20"


bool UBM_Model::initPosition(const string& str)
{
  vector<string> svector = parseString(str, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    svector[i] = tolower(stripBlankEnds(svector[i]));
    string param = biteStringX(svector[i], '=');
    string value = svector[i];

    // Support older style spec "5,10,180,2.0,0" - x,y,hdg,spd,dep
    if(value == "") {
      value = param;
      if(i==0)      param = "x";
      else if(i==1) param = "y";
      else if(i==2) param = "heading";
      else if(i==3) param = "speed";
      else if(i==4) param = "depth";
    }

    double dval  = atof(value.c_str());
    if(param == "x") {
      m_record.setX(dval);
      m_record_gt.setX(dval);
    }
    else if(param == "y") {
      m_record.setY(dval);
      m_record_gt.setY(dval);
    }
    else if((param == "heading") || (param=="deg") || (param=="hdg")) {
      m_record.setHeading(dval);
      m_record_gt.setHeading(dval);
    }
    else if((param == "speed") || (param == "spd")) {
      m_record.setSpeed(dval);
      m_record_gt.setSpeed(dval);
    }
    else if((param == "depth") || (param == "dep")) {
      m_record.setDepth(dval);
      m_record_gt.setDepth(dval);
    }
    else
      return(false);
  }
  return(true);
}

//------------------------------------------------------------------------
// Procedure: propagateNodeRecord

void UBM_Model::propagateNodeRecord(NodeRecord& record,
				    double delta_time, 
				    bool apply_external_forces)
{
  double prior_spd = record.getSpeed();
  double prior_hdg = record.getHeading();

  m_sim_engine.setThrustModeReverse(m_thrust_mode_reverse);

  if(m_thrust_mode == "differential") {
    m_sim_engine.propagateSpeedDiffMode(record, m_thrust_map, delta_time,
					m_thrust_lft, m_thrust_rgt, m_max_acceleration,
                                        m_max_deceleration,m_friction_coef);
    m_sim_engine.propagateHeadingDiffMode(record, delta_time, m_thrust_lft, 
					  m_thrust_rgt, m_turn_rate, 
					  m_rotate_speed);
  }
  else {
    m_sim_engine.propagateSpeed(record, m_thrust_map, delta_time,
				m_thrust, m_rudder, m_max_acceleration,
                                m_max_deceleration,m_friction_coef);
    m_sim_engine.propagateHeading(record, delta_time, m_rudder, 
                                  m_thrust, m_turn_rate);
  }


  m_sim_engine.propagateDepth(record, delta_time, 
			      m_elevator, m_buoyancy_rate, 
			      m_max_depth_rate,
			      m_max_depth_rate_speed);

  // Calculate the total external forces on the vehicle first.
  double total_drift_x = 0;
  double total_drift_y = 0;
  
  if(apply_external_forces) {
    total_drift_x = m_drift_x;
    total_drift_y = m_drift_y;
  }

  m_sim_engine.propagate(record, delta_time, prior_hdg, prior_spd,
                         total_drift_x, total_drift_y,m_rudder);

  // If m_water_depth > 0 then something is known about the present
  // water depth and thus we update the vehicle altitude.
  if(m_water_depth > 0) {
    double depth = record.getDepth();
    double altitude = m_water_depth - depth;
    if(altitude < 0) 
      altitude = 0;
    record.setAltitude(altitude);
  }
}

//------------------------------------------------------------------------
// Procedure: getDriftMag

double UBM_Model::getDriftMag() const
{
  return(hypot(m_drift_x, m_drift_y));
}

//------------------------------------------------------------------------
// Procedure: getDriftAng

double UBM_Model::getDriftAng() const
{
  return(relAng(0, 0, m_drift_x, m_drift_y));
}




