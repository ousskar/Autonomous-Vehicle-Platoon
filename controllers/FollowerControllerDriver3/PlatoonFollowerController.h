/*
 * PlatoonFollowerController.h
 *
 *  Created on: Apr 27, 2015
 *      Author: oussama
 */
#ifndef PLATOONFOLLOWERCONTROLLER_H_
#define PLATOONFOLLOWERCONTROLLER_H_
#include <iostream>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Driver.hpp>
#include <sstream>
#include <string>

using namespace std;
using namespace webots;


typedef struct
   {
   double x ;
   double y ;
   } POSITION ;

class FollowerControllerDriver : public Driver {
  private:
      double oldValueSteer;
      double integralSteer;
      double oldValueSpeed;
      double integralSpeed;
      Compass *compass;
      GPS *gps;
      Emitter *emitter;
      Receiver *receiver;
      Camera *camera;
      Display *display;  
      double angle;
      double steer;
      double speed;
      double current_speed;
      double distance;  // distance between the leader and follower 
      POSITION position; 
      //define initial positions
      double x_initial,y_initial;
      // Received data 
      double xr,yr,x_ref,y_ref;
      double status;
      double LV_angles[1000];
      double LV_steers[1000];
      double LV_speeds[1000];
      clock_t LV_times[1000];
      double LV_current_speeds[1000];
      POSITION LV_positions[1000];   
      int k,k2;
      string robotName;
      bool start_iteration;
      bool following_started;
      double apply_LV_references;
      bool apply_LV_references_started;
      bool leader_position_reached;
      int diff_messages; // messages received before arriving to leader position 
      int channel;
      // camera   
      int camera_width;
      int camera_height;
      double camera_fov;
      // SICK laser
      int sick_width;
      double sick_range;
      double sick_fov;
      int *ptf,*ptl; /*ptf pointer to last applied information from follower 
                     *ptl pointer to the last information received from leader  */
      clock_t startTimer; 
      stringstream logstream; // the log message to be printed in log file "log.txt" 
      bool brake_is_on;
      
           
  public:
    FollowerControllerDriver(); // class Constructor     
    ~FollowerControllerDriver(); // Class Destructor
    void setSpeed(double kmh); // Set target speed
    void setSteer(double str); // Set steering angle
    void receiveNextVehicle(); // Receive Data From Leader
    void sendRearVehicle(); // Send Data to the next follower
    void getVehicleInfo(); // Get vehicle informations(steer,speed...etc)
    void normalMode(); // Run Normal Mode  Behavior
    void adjustment(); // Make Adjustment using Sicklms
    double  process_sick_data(const float *sick_data, double * obstacle_dist); // get angle made with leader  
    void setCommunicationChannelByRobotName(); // set emitter channel for vehicle
    double PIDControlSpeed(double distance,double safeDistance,double speed1,double speed2,double currentSpeed);  //PID Control : Maintain Safe Distance Between Vehicles
    double PIDsetSteer(double setpoint,double vehicle_angle); //  PID Control : to steer the vehicle
    double timer(clock_t start); // Controller Timer
    void sayRobotName(); // Get the Robot Name
    void addLog(string); // Write in log File
    void run(); // initializing and running FollowerController class
};



#endif /* PLATOONFOLLOWERCONTROLLER_H_ */
