/*
 * File:          MyController.cpp
 * Date:          February 20, 2015
 * Description:   Follower vehicle controller 
 * Authors:       Oussama Karoui - www.oussamakaroui.com
 *                Emna Guerfala
 */
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
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

/* PID gains 
*  Safe Distance(KPs,KIs,KDs) + Steer(KPstr,KIstr,KDstr)
*/
double KPs=1.0;
double KIs=0.06; 
double KDs=1.0; 
double KPstr=1.0;
double KIstr=0.08;  
double KDstr=1.0; 

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// to be used as array indices
enum { X, Y, Z };


#define TIME_STEP 50
#define COMMUNICATION_CHANNEL 2
#define UNKNOWN 99999.99
//Distance de reference
#define REFERENCE_DISTANCE 3
#define SAFETY_DISTANCE 7
#define INITIAL_SPEED 60.0

#define KP 1
#define KI 0.006
#define KD 2


typedef struct
   {
   double x ;
   double y ;
   } POSITION ;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
class MyController : public Driver {
  
  // You may need to define your own functions or variables, like
  //  LED *led;
  private:  
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
                            
        
         
  public:
    /*
    *  Constructor
    */
    MyController() : Driver() {
      robotName=getName();
      compass = getCompass("compass");
      gps= getGPS("gps");
      emitter = getEmitter("emitter");
      receiver = getReceiver("receiver");
      camera = getCamera("camera");
      display = getDisplay("display");
      // enable devices
      compass->enable(TIME_STEP);
      gps->enable(TIME_STEP);
      receiver->enable(TIME_STEP);
      camera->enable(TIME_STEP);
      sick_width = camera->getWidth();
      sick_range = camera->getMaxRange();
      sick_fov = camera->getFov();
      k=0;
      k2=0;
      start_iteration=true;
      following_started =true;
      apply_LV_references=false;
      apply_LV_references_started=false;
      leader_position_reached=false;
      diff_messages = 0; 
      status=0;
      // camera
      camera_width = -1;
      camera_height = -1;
      camera_fov = -1.0;
      // SICK laser
      sick_width = -1;
      sick_range = -1.0;
      sick_fov = -1.0;
      ptf=&k2;
      ptl=&k;
      

      
    }
    // MyController destructor
    virtual ~MyController() {
    
    }
    
    /*
    *  Receive data
    */
    void receiveNextVehicle()
    {
      while(receiver->getQueueLength()>0)
      {
        const double* buffer = (double*)receiver->getData();        
        xr = buffer[0];
        yr = buffer[1];
        LV_angles[k]=buffer[2];
        LV_steers[k]=buffer[3];
        LV_speeds[k]=buffer[4];
        LV_current_speeds[k]=buffer[5];
        LV_positions[k].x=xr;
        LV_positions[k].y=yr;
        if(following_started)
        {
          x_ref = xr;
          y_ref = yr;
          following_started =false;
        } 
        cout << "Leader speed =" << LV_speeds[k] << endl;
        k++;
        receiver->nextPacket();
        }
      }
      
      /*
      * Send data
      */
      void sendRearVehicle(){
            double array[7] = {position.x,position.y,angle,steer,speed,current_speed,status};
            emitter->send(array, 7* sizeof(double));
      }
      
      /*
      *  Get vehicle informations(steer,speed...etc)
       */
      void getVehicleInfo()
      {
        const double *north = compass->getValues();
        angle = atan2(north[0],north[2] );
        position.x= gps->getValues()[X];
        position.y=gps->getValues()[Z];  
        speed=getTargetCruisingSpeed();         
        current_speed = getCurrentSpeed();
        steer = getSteeringAngle();
        distance=sqrt(pow(xr-position.x,2)+pow(yr-position.y,2));
        return;
       }
      /*
      * Behavior : Normal Mode 
      */   
       void normalMode()
       {
        double new_angle=0.0;
              int best_k2=floor(k-(distance/k));
              //int best_k22=floor(k-(distance/REFERENCE_DISTANCE));
              if(*ptf<best_k2 && *ptf<(*ptl-1))
              {
                //double PID = applyPID_cruise();
                
                double PID=PIDsetSafeDistance(distance,SAFETY_DISTANCE,LV_speeds[*ptf],LV_speeds[(*ptf)+1],LV_current_speeds[*ptf]);
                setCruisingSpeed(PID);
                //new_angle=applyPID_Angle(LV_angles[k2],angle);
                new_angle=PIDsetSteer(LV_angles[*ptf],angle);
                steer=new_angle;
                setSteeringAngle(new_angle);
                k2++;
              } 
       return;
       }
       
      /*
      * Adjustment using sicklms
      */
      void adjustment()
      {
        double obstacle_dist;
        const float *sick_data = NULL;
        sick_data = camera->getRangeImage();
        double obstacle_angle = process_sick_data(sick_data, &obstacle_dist);      
        if(obstacle_angle>0.2 && obstacle_angle <0.5)
          steer +=0.02;
        else if(obstacle_angle <-0.2 && obstacle_angle > -0.5)
          steer -=0.02;    
        setSteeringAngle(steer);
        if(distance<=SAFETY_DISTANCE){
          setBrake(0.4);
        }else if(obstacle_angle != UNKNOWN){
                      setBrake(0.0);
                 }else setBrake(0.2);  
      }
      
      // returns approximate angle of obstacle
      // or UNKNOWN if no obstacle was detected
      double process_sick_data(const float *sick_data, double * obstacle_dist) 
      { 
        const int HALF_AREA = 20;  // check 20 degrees wide middle area
        int sumx = 0;
        int collision_count = 0;
        int x;
        *obstacle_dist = 0.0;
        for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
          float range = camera->rangeImageGetDepth(sick_data, sick_width, x, 0);
          if (range < 20.0) {
            sumx += x;
            collision_count++;
            *obstacle_dist += range;
            //printf("x   %d \n", x);
          }
        }
             //printf("collision count  %d \n", collision_count);
           //  printf("distance  %f,real distance=%lf \n", *obstacle_dist,distance);
        // if no obstacle was detected...
        if (collision_count == 0)
          return UNKNOWN;
        
        *obstacle_dist = *obstacle_dist / collision_count;
        return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
      }
      
      void setCommunicationChannelByRobotName()
      {
      switch(robotName[8]){
        case '1':
          channel=2;        
          break;
        case '2':
          channel=3; 
          break;
        case '3':
          channel=4;
          break;
        case '4':
          channel=5;
          break;          
      }
      emitter->setChannel(channel);  
      }
               
    // User defined function for initializing and running
    // the MyController class
    void run() {
      //cout << " The robot name =" << robotName << endl;
      setCruisingSpeed(INITIAL_SPEED); // Initiate the speed
      
      setCommunicationChannelByRobotName();
      

      
      
      // Main loop:
      // Perform simulation steps of 64 milliseconds
      // and leave the loop when the simulation is over
      while (step() != -1) {  
      
        static int i =0;
         if(i%(int)(TIME_STEP / getBasicTimeStep()) == 0)
         { 
            getVehicleInfo(); // Get vehicle informations
            receiveNextVehicle(); // Receive data

            if(!leader_position_reached)
            {
              if(sqrt(pow(x_ref-position.x,2)+pow(y_ref-position.y,2))<0.5)
              {
                leader_position_reached=true;
                cout << "leader position reached " << endl;
              }
            }
            
            if(leader_position_reached)
            {
              if(*ptf<*ptl)
              {
                if(start_iteration)
                {
                  x_initial = position.x;
                  y_initial = position.y; 
                  start_iteration=false;
                 }
               }
              
              // Use timer
              double dis=sqrt(pow(position.x-x_initial,2)+pow(position.y-y_initial,2));           
              if(dis >= REFERENCE_DISTANCE){ 
               x_initial = position.x;
               y_initial = position.y;  
               start_iteration=true;
              sendRearVehicle(); // Send data to follower
              }
               normalMode(); // Normal Mode Behavior
               adjustment(); // Make some adjustments using sicklms
                 
                
                
            } //End if
                 
            
 
         } 
            
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
  MyController* controller = new MyController();
  controller->run();
  delete controller;
  return 0;
}




