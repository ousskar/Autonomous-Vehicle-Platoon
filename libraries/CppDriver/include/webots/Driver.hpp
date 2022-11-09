/*
 * File:          Driver.hpp
 * Date:          September 2014
 * Description:   CPP wrapper of the driver library
 * Author:        david.mansolino@epfl.ch
 * Modifications: 
 * Comments:      Sponsored by the CTI project RO2IVSim (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <webots/Robot.hpp>

namespace webots {
  class Driver : public Robot {
    public:
      enum {
        INDICATOR_OFF=0, 
        INDICATOR_RIGHT,
        INDICATOR_LEFT
      };
    
      Driver();
      virtual ~Driver();
      
      virtual int step();
      
      // positive: turn right, negative: turn left
      void   setSteeringAngle(double steeringAngle);
      double getSteeringAngle();
      
      void   setCruisingSpeed(double speed);
      double getTargetCruisingSpeed();
      
      double getCurrentSpeed();
      
      void   setThrottle(double throttle);
      double getThrottle();
      
      void   setBrake(double brake);
      double getBrake();
      
      void setIndicator(int state);
      void setIndicatorWarning(bool state);
      
      void setDippedBeams(bool state);
      void setAntifogLights(bool state);
      
      bool getDippedBeams();
      bool getAntifogLights();
      
      double getRpm();
      int    getGear();
      void   setGear(int gear);
      int    getGearNumber();
      
    private:
      virtual int step(int t) { return Robot::step(t); }
  };
}

#endif // DRIVER_HPP
