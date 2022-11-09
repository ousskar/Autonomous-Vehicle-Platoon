/*
 * PlatoonFollowerController.cpp
 *
 *  Created on: Apr 27, 2015
 *      Author: oussama
 */

#include "PlatoonFollowerController.h"
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Driver.hpp>
#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Compass.hpp>
#include <ctime>
#include <string>
#include <fstream>
#include <sstream>

#define TIME_STEP 50
#define UNKNOWN 99999.99
//Distance de reference
#define REFERENCE_DISTANCE 3
#define SAFETY_DISTANCE 7
#define INITIAL_SPEED 50.0

// to be used as array indices
enum { X, Y, Z };
using namespace std;
using namespace webots;

/* PID gains 
*  Safe Distance(KPs,KIs,KDs) + Steer(KPstr,KIstr,KDstr)
*/
double KPs=2.0;    // 2 : 0.005 : 2.0
double KIs=0.005; 
double KDs=2.0; 
double KPstr= 0.7;         //0.7 : 0.006 :1.0
double KIstr=0.006;          // Test: (0.8: 0.006 : 1.0)  
double KDstr=0.0; 

FollowerControllerDriver :: FollowerControllerDriver() :  Driver(){
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
      start_timer=true;
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
      oldValueSteer=0.0;
      integralSteer=0.0;
      oldValueSpeed=0.0;
      integralSpeed=0.0;
      delay=0.0;
      startTimer = clock();
      brake_is_on=false;
      normal_mode=true;
      oldrs=0.0;
      klog=0;
  }
  // FollowerControllerDriver destructor
  FollowerControllerDriver :: ~FollowerControllerDriver() {
    
  }
  
  /*
  * Set Speed
  */  
  void FollowerControllerDriver :: setSpeed(double kmh)
  { 
   speed = kmh;
   setCruisingSpeed(speed);  
  }
  
  /*
  * Set Steering Angle
  */
  void FollowerControllerDriver :: setSteer(double str)
  {
    steer = str;
    // limit range of the steering angle
    if (str > 0.5)
      str = 0.5;
    else if (str < -0.5)
      str = -0.5;
    setSteeringAngle(str);
  }
  
   /*
    *  Receive data
    */
    void FollowerControllerDriver :: receiveNextVehicle()
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
        LV_times[k]=clock();
        if(following_started)
        {
          x_ref = xr;
          y_ref = yr;
          following_started =false;
        } 
        k++;
        receiver->nextPacket();
        }
      }
      
      /*
      * Send data
      */
      void FollowerControllerDriver :: sendRearVehicle(){
            double array[7] = {position.x,position.y,angle,steer,speed,current_speed,status};
            emitter->send(array, 7* sizeof(double));
      }
       /*
      *  Get vehicle informations(steer,speed...etc)
       */
      void FollowerControllerDriver :: getVehicleInfo()
      {
        const double *north = compass->getValues();
        angle = atan2(north[0],north[2] );
        position.x= gps->getValues()[X];
        position.y=gps->getValues()[Z];  
        //speed=getTargetCruisingSpeed();         
        current_speed = getCurrentSpeed();
        //steer = getSteeringAngle();
        distance=sqrt(pow(xr-position.x,2)+pow(yr-position.y,2));
        return;
       }
      /*
      * Behavior : Normal Mode 
      */   
       void FollowerControllerDriver :: setNormalMode()
       {  
         double new_angle=0.0;
         double ptimer =timer(startTimer);
         //double distanceNextPosition=sqrt(pow(position.x-LV_positions[k2+1].x,2)+pow(position.y-LV_positions[k2+1].y,2));
         //if(current_speed>0)
          //delay = (distanceNextPosition/((current_speed*1000)/3600000)); 
          //cout << delay << endl;
         //delay +=0.00000001; 
         double T=  1.0;  
         int bestVal=floor(k-(distance/k));
         if((k2<bestVal) && (ptimer >= T)){
           /*
           //LOG POSITIONS
           ostringstream strs;
           strs << position.x <<","<<position.y << ":" << LV_positions[k2].x << "," <<LV_positions[k2].y ;
           string str = strs.str();
           addLog(str);
           */
           if(robotName=="follower2"){
             ostringstream strs;
             strs << distance;
             string str = strs.str();
             addLog(str);
           }
           //delay= (distance/((current_speed*1000)/216000))/( CLOCKS_PER_SEC / 1000 ); 
          //cout << "distance="<<distance<< " delay =" << delay << endl;
          double PID=PIDControlSpeed(distance,SAFETY_DISTANCE,LV_speeds[*ptf],LV_speeds[(*ptf)+1],LV_current_speeds[*ptf]);
          setSpeed(PID);
          new_angle=PIDsetSteer(LV_angles[*ptf],angle);
          setSteer(new_angle);
          start_timer=true;
          k2++; 
          }
           if(distance<=SAFETY_DISTANCE)
           {
           brake_is_on =true;
           setBrake(0.4);
           }
          
           if((distance>SAFETY_DISTANCE) || (brake_is_on==true)){
             setBrake(0.0);
             brake_is_on =false;             
           }

            //Calculate the distance travelled
            double dis=sqrt(pow(position.x-x_initial,2)+pow(position.y-y_initial,2));           
            if(dis >= REFERENCE_DISTANCE){ 
             x_initial = position.x;
             y_initial = position.y;  
             start_iteration=true;
             sendRearVehicle();
            }
       }
       
       
      /*
      * Behavior : Degraded Mode 
      */   
       void FollowerControllerDriver :: setDegradedMode()
       {  
         
       }
       
       
       /*
       * Change the Platoon Behavior
       */    
       /*     
        void setMode(double code)
        {
          switch(code){
            case '1':  // Full Brake
              setSpeed(0.0);
              setBrake(0.4);
              normal_mode=false;
              status=1.0;
              sendRearVehicle(); // Inform other vehicles 
              break;
          }
        }
        */
       
       
      /*
      * Adjustment using sicklms
      */
      void FollowerControllerDriver :: adjustment()
      {
        double obstacle_dist;
        const float *sick_data = NULL;
        sick_data = camera->getRangeImage();
        double obstacle_angle = process_sick_data(sick_data, &obstacle_dist); 
        /* 
        if(obstacle_angle>0.2 && obstacle_angle <0.5)
          steer = steer + obstacle_angle / obstacle_dist;
        else if(obstacle_angle <-0.2 && obstacle_angle > -0.5)
          steer = steer + obstacle_angle  / obstacle_dist;   
        */
        setSteer(steer);   
        if(distance<=SAFETY_DISTANCE){
          setBrake(0.4);
        }else if(obstacle_angle != UNKNOWN){
          setBrake(0.0);
        }else setBrake(0.2);  
      }
      
      // returns approximate angle of obstacle
      // or UNKNOWN if no obstacle was detected
      double FollowerControllerDriver :: process_sick_data(const float *sick_data, double * obstacle_dist) 
      { 
        const int HALF_AREA = 40;  // check 20 degrees wide middle area
        int sumx = 0;
        int collision_count = 0;
        int x;
        *obstacle_dist = 0.0;
        for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
          float range = camera->rangeImageGetDepth(sick_data, sick_width, x, 0);
          if (range < 30.0) {
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
      
      void FollowerControllerDriver :: setCommunicationChannelByRobotName()
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
        case '5':
          channel=6;
          break;
        case '6':
          channel=7;
          break;          
      }
      emitter->setChannel(channel);  
      }
      
      
  
       /*
       *  PID Control : Maintain Safe Distance Between Vehicles
       */
      double FollowerControllerDriver :: PIDControlSpeed(double distance,double safeDistance,double speed1,double speed2,double currentSpeed)
      {
        double Vfollowed=LV_speeds[k-1];
        double result=0.0;
        
        double err = distance - SAFETY_DISTANCE;
        double diff= err-oldValueSpeed;
        
        if(floor(abs(err))==0)
          integralSpeed=0.0;
        
          // limit integral
        if (integralSpeed < 20)
          integralSpeed += err;
      
      
        oldValueSpeed = err;
        result= ((KPs * err) + Vfollowed) + (KIs * integralSpeed) + (KDs * diff); 
        return result;    
    }
    
    
    /*
    * PID Control : to steer the vehicle
    */
    double FollowerControllerDriver :: PIDsetSteer(double setpoint,double vehicle_angle)
    {
        double rs = 0.0; // the output of PID
        setpoint = LV_angles[k2];
        // anti-windup mechanism
        /*
        if (((signbit(setpoint) != signbit(vehicle_angle))&&(abs(setpoint-LV_angles[k2-1])>3.14))|| (abs(setpoint-vehicle_angle)>3)){
          rs=LV_steers[k2];
          return rs;
        } 
        */
        if ((signbit(setpoint) != signbit(vehicle_angle)) && (vehicle_angle <0)){
         vehicle_angle += 2*M_PI;
        } else if ((signbit(setpoint) != signbit(vehicle_angle))  && (vehicle_angle >0)){
         vehicle_angle = 0;
        }
         double error = setpoint - vehicle_angle;
         

        // limit integral
          if((integralSteer > -10) && (integralSteer < 10)) 
            integralSteer += error;          

        
        double diff = error - oldValueSteer;
        oldValueSteer = error;
        rs = KPstr * error +  KIstr * integralSteer + KDstr * diff;
        oldrs=rs;
        return rs;
    }

  /*
  *   Timer to fix the next sending time
  *   @ return period of time
  */
  double FollowerControllerDriver :: timer(clock_t start){
    double duration;
    duration = (clock()-start)/(CLOCKS_PER_SEC/1000);
    return duration;
  }

  
  void FollowerControllerDriver ::  sayRobotName(){
   cout << "Robot Name " << robotName << endl ; 
  }
  
  
   void FollowerControllerDriver :: addLog(string str)
   {
    ofstream mylogfile ("log.txt",ios_base::app);
    if (mylogfile.is_open())
    {
      mylogfile << str << ",";
      mylogfile.close();
    }
    else cout << "unable to open file"<<endl;
   } 
  
  
    // User defined function for initializing and running
    // FollowerController class
    void FollowerControllerDriver :: run()
    {
      //cout << " The robot name =" << robotName << endl;
      setCruisingSpeed(INITIAL_SPEED); // Initiate the speed 
      setCommunicationChannelByRobotName();
      // Delete recent log file content
      
      std::ofstream ofs;
      ofs.open("log.txt", ofstream::out | ofstream::trunc);
      ofs.close();     
      
      
      // Main loop:
      // Perform simulation steps of 64 milliseconds
      // and leave the loop when the simulation is over
      while (step() != -1) 
      {  
      
        static int i =0;
         if(i%(int)(TIME_STEP / getBasicTimeStep()) == 0)
         {  
            getVehicleInfo(); // Get vehicle informations
            receiveNextVehicle(); // Receive data
             
              if(!leader_position_reached){
               
                if(sqrt(pow(x_ref-position.x,2)+pow(y_ref-position.y,2))<0.5){
                  leader_position_reached=true;
                  diff_messages = k-1; // messages received before arriving to leader position
                  printf("leader position reached \n");
                }
              }
               if(leader_position_reached){
                 if(k2<k)
                 {
                  if(start_iteration){
                    x_initial = position.x;
                    y_initial = position.y; 
                    start_iteration=false;
                  }
                  if(start_timer)
                  {
                    startTimer=clock();
                    start_timer=false;
                  }         
                 }
                 if(normal_mode)
                   setNormalMode();
                 adjustment();
 
         } 
            
      }
    }
    }
    
