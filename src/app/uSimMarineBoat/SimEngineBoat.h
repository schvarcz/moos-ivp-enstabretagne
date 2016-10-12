/*****************************************************************/
/*    NAME: Dominique Monnet                                     */
/*    ORGN: ENSTA-Bretagne                                       */
/*    FILE: SimEngineBoat.cpp                                    */
/*    DATE: Sep 27, 2016                                         */
/*                                                               */
/*****************************************************************/

#ifndef SIM_ENGINE_BOAT_HEADER
#define SIM_ENGINE_BOAT_HEADER

#include "NodeRecord.h"
#include "ThrustMap.h"

class SimEngineBoat
{
public:
  SimEngineBoat() {}
  ~SimEngineBoat() {}
  
 public:
  void setThrustModeReverse(bool v) {m_thrust_mode_reverse=v;}

public:
  void propagate(NodeRecord&, double delta_time, double prior_heading,
                 double prior_speed, double drift_x, double drift_y,double rudder);
  
  void propagateDepth(NodeRecord&, double delta_time, 
		      double elevator_angle, double buoyancy_rate, 
		      double max_depth_rate, 
		      double m_max_depth_rate_speed);

  void propagateSpeed(NodeRecord&, const ThrustMap&, double delta_time, 
		      double thrust, double rudder,
                      double max_accel, double max_decel, double friction);

  void propagateHeading(NodeRecord&, double delta_time, double rudder,
                        double thrust, double turn_rate);

  // Differential Thrust Modes
  void propagateSpeedDiffMode(NodeRecord&, const ThrustMap&, double delta_time, 
			      double thrust_left, double thrust_right,
                              double max_accel, double max_decel,double friction);
  
  void propagateHeadingDiffMode(NodeRecord&, double delta_time, double rudder,
				double thrust_left, double thrust_right, 
				double rotate_speed);

 protected:
  bool m_thrust_mode_reverse;
};

#endif
