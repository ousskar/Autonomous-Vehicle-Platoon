// File:          AutonomousController.cpp
// Date:          
// Description:   
// Author:        
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Driver.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Compass.hpp>
#include "ControllerFunctions.h"
#include <cmath>
#include <string>
#include <iostream>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

#define TIME_STEP 50

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
class AutonomousController : public Driver {
  
  // You may need to define your own functions or variables, like
  //  LED *led;
  Compass *compass;
  GPS *gps;
  Emitter *emitter;
  Receiver *receiver;
  Camera *camera;
  Display *display;

  
  
  public:
    // AutonomousController constructor
    AutonomousController() : Driver() {

      
      // You should insert a getDevice-like function in order to get the
      // instance of a device of the robot. Something like:
      //  led = getLED("ledName");
      compass = getCompass("compass");
      gps= getGPS("gps");
      emitter = getEmitter("emitter");
      receiver = getReceiver("receiver");
      camera = getCamera("camera");
      display = getDisplay("display");
      
      
      //wb_compass_enable (compass, TIME_STEP);
      
    }


    // AutonomousController destructor
    virtual ~AutonomousController() {
      
      // Enter here exit cleanup code
    }
    
    // User defined function for initializing and running
    // the AutonomousController class
    void run() {
      
      // Main loop:
      // Perform simulation steps of 64 milliseconds
      // and leave the loop when the simulation is over
      while (step() != -1) {
      
      
      
      //wbu_driver_set_speed(50);
       //setCruisingSpeed(60.0);
       // double speed=wbu_driver_getCurrentSpeed();
        //cout << "speed = "<<speed;
          
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = distanceSensor->getValue();
        
        // Process sensor data here
        
        // Enter here functions to send actuator commands, like:
        //  led->set(1);
      }
    }
};

// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  AutonomousController* controller = new AutonomousController();
  controller->run();
  delete controller;
  return 0;
}
