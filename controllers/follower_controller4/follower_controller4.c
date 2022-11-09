/*
 * File:          follower_controller2.c
 * Date:          February 20, 2015
 * Description:   Follower vehicle controller example
 * Authors:       Oussama Karoui - www.oussamakaroui.com
 *                Emna Guerfala
 */

#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/driver.h>
#include <webots/display.h>
#include <webots/compass.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <time.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

//------------------------------------------------------------------Variables definition

// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 50
#define COMMUNICATION_CHANNEL 5
#define UNKNOWN 99999.99
// Line following PID

//Distance de reference
#define REFERENCE_DISTANCE 3
#define SAFETY_DISTANCE 7
#define INITIAL_SPEED 50.0

#define KP 1
#define KI 0.006
#define KD 2




#define WHEEL_DIAMETER 0.748
#define AXLES_DIST 2.995
#define AXLE_LENGTH 1.627

// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool reference_distance = false;


// devices
WbDeviceTag left_front_wheel, right_front_wheel;
WbDeviceTag left_steer, right_steer;
// compass
WbDeviceTag compass;
//emitter
WbDeviceTag  emitter_communication;
//receiver
WbDeviceTag communication;

// speedometer
WbDeviceTag display;
WbImageRef speedometer_image = NULL;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;
// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;
//platoon variables


typedef struct
   {
   double x ;
   double y ;
  // int k;
   } POSITION ;

POSITION LV_positions[1000];   

double D2=0.0;
double D=0.0;
double LV_angle=0.0;
double xr=0.0; 
double zr=0.0;
double yr=0.0; 




double speed=0.0;
double current_speed=0.0;
POSITION position={0,0};
double steer=0.0; 
double angle=0.0; 
bool brake_is_on=false;



//define initial positions
 double x_initial =0.0;
 double y_initial =0.0;
 double x_ref =0.0;
 double y_ref =0.0;
 bool start_iteration=true;
 bool following_started =true;
 double apply_LV_references=false;
 bool apply_LV_references_started=false;
 bool leader_position_reached=false;
 int diff_messages = 0; // messages received before arriving to leader position

 

double LV_speeds[1000]; // array contains previous speeds of Leading Vehicle 
double LV_steers[1000];
double LV_angles[1000]; // array contains previous steers of Leading Vehicle 
double LV_current_speeds[1000];
double LV_current_speed=0.0;
int k=0;
int k2=0;

double steering_angle = 0.0;
double previous_steering_angle=0.0;
double distance=0.0;


/*
*    Platoon Behavior
*    0 - Normal status
*    * - Emergency status dependant to emergency codes
*/
double status=0;


/*
* Emergency variables
*/

// Emergency variables
bool FB_em=false;  // Full brake emergency code 1
//emergency codes
double FB_code = 1;

/*
*   Functions used 
*/
void set_speed(double kmh);
void set_steering_angle(double wheel_angle);
void call_emergency(int code);
void receive_data_next_vehicle();
void send_data_rear_vehicle();
double process_sick_data(const float *sick_data, double * obstacle_dist);


int get_nearest_position(int p1,int p2);

//------------------------------------------PID control distance
//  safety distance gains   1/0.05/2
/*
#define KPs 1
#define KIs 0.06
#define KDs 2
*/
double applyPID_cruise() {
  double KPs=1.0;
  double KIs=0.06;  //0.08
  double KDs=1.0; 
  double Vfollowed=0.0;
  if(LV_speeds[k2]==LV_speeds[k2+1]){
    Vfollowed=LV_speeds[k2];
  }else Vfollowed=LV_current_speeds[k2]; 
  
  if(Vfollowed<80){
  KPs=2.0;
  KIs=0.2;
  }
  
  static double oldValue = 0.0;
  static double integral = 0.0;
  double result=0.0;
  
  double err = distance - SAFETY_DISTANCE;
  double diff= err-oldValue;
  
  if(floor(abs(err))==0){
    integral=0.0;
    result= Vfollowed;
    return result;
    }
  
    // limit integral
  if (integral < 10)
    integral += err;
 oldValue = err;
result= ((KPs * err) + Vfollowed) + (KIs * integral) + (KDs * diff); 
/*
printf("-----------\n");
printf("Leader speed = %lf\n", Vfollowed);
printf("Apply PID = %lf\n",result);
printf("distance=%lf\n",distance);   
printf("-----------\n");
*/
return result;
}

//PID controller   1/0.06/1 2/0.002/1  1/0.01/1
/*
#define KPstr 1
#define KIstr 0.08//0.02
#define KDstr 1
*/
/*
double atan2pi(double x){
  x > 0 ? x : (2*M_PI + x); 
  //x*=M_PI/180;
return x;
}
*/



double applyPID_Angle(double vehicle_angle,double obstcl_angle) {
  double KPstr=1.0;
  double KIstr=0.06;  //0.08
  double KDstr=1.0; 
  double rs = 0.0; // the output of PID
  static double oldrs=0.0;
  double theta_dot=0.0;
  double setpoint = LV_angles[k2];
  static double oldValue = 0.0;
  static double integral = 0.0;

 

  
 // printf("setpoint=%lf,previous setpoint=%lf,vehicle angle=%lf\n",setpoint,LV_angles[k2-1],vehicle_angle);
  /*
  double new =0.0;
  double new2=0.0;
  new = atan2pi(setpoint);
  new2= atan2pi(vehicle_angle);
  printf("new=%lf,new vehicle_angle=%lf\n",new,new2);
  */
  
  // anti-windup mechanism
  if ((signbit(setpoint) != signbit(vehicle_angle))){
   vehicle_angle += 2*M_PI;
   //   if(vehicle_angle<0)
   //   vehicle_angle *=;
   //printf("setpoint=%lf,previous setpoint=%lf,new vehicle angle=%lf\n",setpoint,LV_angles[k2-1],vehicle_angle);
  //  rs=LV_steers[k2];
   // return rs;
    //integral = 0.0;
  } 
  
  
  
   double error = setpoint - vehicle_angle; 
//  if(floor(fabsobstcl_angle
 
  double cst=0.02;
  double Vleader = LV_current_speeds[k-1];
  theta_dot = fabs((cst*error*Vleader)/distance);
  printf("theta_dot=%lf, current_speed =%lf\n",theta_dot, current_speed);
  if(Vleader<80)
    KIstr=0.006; 
  // PID Gains
  //double KPstr=1.0;
 // KIstr=theta_dot;  //0.08
  //double KDstr=1.0; 
  
 // printf("error=%lf,Vleader=%lf,theta_dot=%lf,distance=%lf\n",error,Vleader,theta_dot,distance);
 
  // limit integral
  if (integral < 30 && integral > -30)
    integral += error;
    
    
  
  oldValue = error;
  double diff = error - oldValue;
  rs = KPstr * error +  KIstr * integral + KDstr * diff;
  oldrs=rs;
  /*
  printf("------------\n");
  printf("---->>>LV_angle=%lf,vehicle_angle=%lf,integral=%lf,error=%lf\n",setpoint,vehicle_angle,integral,error);
  printf("---->>>LV_steer=%lf,steer =%lf,PID STEER=%lf\n",LV_steers[k2],steer,rs);
  */
  return rs;
}


double applyPID(double yellow_line_angle);


void normalMode(){  
       
//Calculate the distance travelled
double dis=sqrt(pow(position.x-x_initial,2)+pow(position.y-y_initial,2));           
if(dis >= REFERENCE_DISTANCE){ 
 x_initial = position.x;
 y_initial = position.y;  
 start_iteration=true;
//Send data to next follower
send_data_rear_vehicle();
}
}




void update_display();

//**********************  Main ***********************************
int main(int argc, char **argv)
 
  {
  int channel;
      wbu_driver_init();
	  
    //receiver
       communication = wb_robot_get_device("receiver");
       wb_receiver_enable(communication, TIME_STEP);
      
    
  //emitter
  
  emitter_communication = wb_robot_get_device("emitter");
  channel = wb_emitter_get_channel(emitter_communication);
    if (channel != COMMUNICATION_CHANNEL) {
      wb_emitter_set_channel(emitter_communication, COMMUNICATION_CHANNEL);
    }


  // check if there is a SICK and a display
  int j = 0;
  for (j=0; j<wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char * name = wb_device_get_name(device);
    if (strcmp(name,"lms291") == 0)
      enable_collision_avoidance = false;
    else if (strcmp(name,"display") == 0)
      enable_display = true;
  }
  
        // Sick Sensor
      sick = wb_robot_get_device("lms291");
      wb_camera_enable(sick, TIME_STEP);
      sick_width = wb_camera_get_width(sick);
      sick_range = wb_camera_get_max_range(sick);
      sick_fov = wb_camera_get_fov(sick);
      
   
      //compass
      compass = wb_robot_get_device("compass");
      wb_compass_enable (compass, TIME_STEP);

      // initialize gps
      gps = wb_robot_get_device("gps");
      wb_gps_enable(gps, TIME_STEP);
      
      // camera device
      camera = wb_robot_get_device("camera");
      wb_camera_enable(camera, TIME_STEP);
      camera_width = wb_camera_get_width(camera);
      camera_height = wb_camera_get_height(camera);
      camera_fov = wb_camera_get_fov(camera);
  
           // initialize display (speedometer)
      if (enable_display) {
        display = wb_robot_get_device("display");
        speedometer_image = wb_display_image_load(display, "speedometer.png");
        wb_display_image_paste(display, speedometer_image, 0, 0);
        wb_display_set_color(display, 0xffffff);
        wb_display_draw_text(display, "GPS coords:", 10, 130);
        wb_display_draw_text(display, "GPS speed:", 10, 140);
      }
      
      
      
      

     // start engine
      set_speed(INITIAL_SPEED); // km/h
    
      wbu_driver_set_indicator_warning(true);
      wbu_driver_set_dipped_beams(true);
      wbu_driver_set_antifog_lights(true);





        while (wbu_driver_step() != -1) {     
          static int i = 0;
          
               // updates sensors only every TIME_STEP milliseconds
              //wb_robot_get_basic_time_step == value of basicTimeStep
          if(i%(int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0)
          {   

              const double *north = wb_compass_get_values(compass);
              angle = atan2(north[0],north[2] ); 
              steer =wbu_driver_get_steering_angle();
              current_speed = wbu_driver_get_current_speed();              
              position.x=wb_gps_get_values(gps)[X];
              position.y=wb_gps_get_values(gps)[Z];
              distance=sqrt(pow(xr-position.x,2)+pow(yr-position.y,2));
              
              receive_data_next_vehicle();

              // read sensors              
             const float *sick_data = NULL;
             sick_data = wb_camera_get_range_image(sick);
             double obstacle_dist;
             double obstacle_angle;
  
      
               
               
               
              
               
             
 
              if(!leader_position_reached){
              
                if(sqrt(pow(x_ref-wb_gps_get_values(gps)[X],2)+pow(y_ref-wb_gps_get_values(gps)[Z],2))<0.5){
                  leader_position_reached=true;
                  diff_messages = k-1; // messages received before arriving to leader position
                  printf("leader position reached \n");
                }
              }
               if(leader_position_reached){
               
                if(k2<k){
                  if(start_iteration){
                  x_initial = position.x;
                  y_initial = position.y; 
                  start_iteration=false;
                  }
                 }
                 
                 
                 if(FB_em){
                   call_emergency(1);
                   send_data_rear_vehicle();
                 }else{               
                 normalMode(); 
        
             
             obstacle_angle = process_sick_data(sick_data, &obstacle_dist);
               double new_angle=0.0;
               int best_k2=floor(k-(distance/k));
               //int best_k22=floor(k-(distance/REFERENCE_DISTANCE));
               if(k2<best_k2 && k2<(k-1)){
                 double PID = applyPID_cruise();
                 wbu_driver_set_cruising_speed(PID);
                 new_angle=applyPID_Angle(angle,obstacle_angle);
                 steer=new_angle;
                 wbu_driver_set_steering_angle(new_angle);
                 
                 //set_steering_angle(new_angle);
                // printf("the new speed = %lf,distance = %lf\n",PID,distance);
                 //set_steering_angle(new_angle);
                 k2++; 
                }
               obstacle_angle = process_sick_data(sick_data, &obstacle_dist);
               
               if(obstacle_angle>0.2 && obstacle_angle <0.5)
                 steer +=0.02;
               else if(obstacle_angle <-0.2 && obstacle_angle > -0.5)
                  steer -=0.02;
                  //LV_steers[k2] -=0.01;
               
               wbu_driver_set_steering_angle(steer);
             
               //printf("obs angle %f \n",obstacle_angle);
             /*
              if(obstacle_angle == UNKNOWN){ // the vehicle is in bad direction 
                wbu_driver_set_brake(0.4);
                if(steering_angle<0){ // the vehicle was turning left and lost the direction 
                  // Go back to the left
                    //wbu_driver_get_steering_angle(new_angle+(-steering_angle*0.02));
               new_angle -= steering_angle*0.02;
                }else {
                
                new_angle -= steering_angle*0.02;
                  // Go back to the right
                  //wbu_driver_get_steering_angle(new_angle+(-steering_angle*0.02));
                }
              //  wbu_driver_set_steering_angle(steer);
             }else {
             
                if(distance<=SAFETY_DISTANCE)
                 {
                 brake_is_on =true;
                 wbu_driver_set_brake(0.4);
            
                 //set_speed(40);
                // k2++;
                 //wbu_driver_set_cruising_speed(LV_current_speeds[k]);
                 } 
                
                 if((distance>SAFETY_DISTANCE) || (brake_is_on==true)){
                   wbu_driver_set_brake(0.0);
                   brake_is_on =false;             
                 }
                 
              if(floor(abs(obstacle_angle*10)==0)){
              new_angle=LV_steers[k2];
              }else applyPID_Angle(angle);
              
             }
             set_steering_angle(new_angle);
             */
            /* 
             if(obstacle_angle == UNKNOWN) {
               wbu_driver_set_brake(0.4);
               steer = steering_angle - 0.1;
               set_steering_angle(steer);
             }
             */
             
            /*
            if(obstacle_angle != UNKNOWN) {
             if (obstacle_angle > 0 && obstacle_angle<0.5)
              // steer = (obstacle_angle - 0.25)/100;
               //wbu_driver_set_brake(0.4);
               wbu_driver_set_steering_angle(0.01);
             else if (obstacle_angle < -0.5)
               //wbu_driver_set_brake(0.4);
                wbu_driver_set_steering_angle(-0.01);
             
             //steer = (obstacle_angle + 0.25)/100;
             //wbu_driver_set_steering_angle(steer); 
               }
              */   
              
              
                 
                 if(distance<=SAFETY_DISTANCE){
                   wbu_driver_set_brake(0.4);
                 }else if(obstacle_angle != UNKNOWN){
                  wbu_driver_set_brake(0.0);
                 }else wbu_driver_set_brake(0.2);
                 
                 
                 
                 
                 
                 
                           
                  //set_speed(applyPID_cruise()); 
               }
 
               
               
               }
               
                             

      if (enable_display)
        update_display();

       }  
    ++i;
  
         }
   wbu_driver_cleanup();

  return 0;  // ignored

}   

//******************         End Main        **********************************



// set target speed
void set_speed(double kmh)
 {
 // max speed
 if (kmh > 250.0)
  kmh = 250.0;
  
  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
    wbu_driver_set_cruising_speed(kmh);
}


// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

// Emergency situations  

void call_emergency(int code)
{
  switch(code){
    case 1:
      printf("Full brake CAR 1 emergency\n");
      set_speed(0);
      wbu_driver_set_brake(0.4);
      FB_em=true;   // Activate the full brake
      break;
  }
}


//Receive next vehicle data
void receive_data_next_vehicle(){
    while (wb_receiver_get_queue_length(communication) > 0)
      {
        const double *buffer= wb_receiver_get_data(communication);
        xr=buffer[0];
        yr=buffer[1];
        LV_angles[k]=buffer[2];
        LV_steers[k]=buffer[3];
        LV_speeds[k]=buffer[4];
        LV_current_speeds[k]=buffer[5];
        LV_positions[k].x=xr;
        LV_positions[k].y=yr;
        if (buffer[6]==1)
        {
          printf("Fb signal received !!!!!!!\n");
          FB_em=true;
          status=1;
        }
        k++;
        if(following_started)
        {
          x_ref = xr;
          y_ref = yr;
          following_started =false;
        } 
        wb_receiver_next_packet(communication);
      } 
}

//Send data to next vehicle
void send_data_rear_vehicle(){   
        double array[7] = {position.x,position.y,angle,steer,speed,current_speed,status};
        wb_emitter_send(emitter_communication, array, 7* sizeof(double));
}




//PID controller
double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;
  
  // anti-windup mechanism
  if (signbit(yellow_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;
  
  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;
  
  oldValue = yellow_line_angle;
  return KP * yellow_line_angle +  KI * integral + KD * diff;
}

int get_nearest_position(int p1,int p2){
while(p1<p2-1){
double D1=sqrt(pow(LV_positions[p1].x-wb_gps_get_values(gps)[X],2)+pow(LV_positions[p1].y-wb_gps_get_values(gps)[Z],2));
  if(D1<0.2)
  break;
  
double D2=sqrt(pow(LV_positions[p1+1].x-wb_gps_get_values(gps)[X],2)+pow(LV_positions[p1+1].y-wb_gps_get_values(gps)[Z],2));
  if(D2>D){
  return p1;
  break;
  }
p1++;  
}
return p1;
}

// display the vehicle coordinates and its speed

void update_display() {
  const double NEEDLE_LENGTH = 50.0;
  static int x = 0, y = 0;
  static char txt1[64] = "0.0 0.0";
  static char txt2[64] = "0.0";
  
  // clean display
  wb_display_set_color(display, 0x000000);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);
  wb_display_draw_text(display, txt1, 100, 130);
  wb_display_draw_text(display, txt2, 100, 140);

  // draw speedometer needle
  wb_display_set_color(display, 0xffffff);
  double speed = wbu_driver_get_current_speed();
  if(isnan(speed))
    speed = 0.0;
  double alpha = speed / 260.0 * 3.72 - 0.27;
  x = -NEEDLE_LENGTH * cos(alpha);
  y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y); 

  // draw text
  sprintf(txt1, "%.1f %.1f", gps_coords[X], gps_coords[Z]);
  sprintf(txt2, "%.1f", gps_speed);
  wb_display_draw_text(display, txt1, 100, 130);
  wb_display_draw_text(display, txt2, 100, 140);
}


// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
double process_sick_data(const float *sick_data, double * obstacle_dist) { 
  const int HALF_AREA = 20;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = wb_camera_range_image_get_depth(sick_data, sick_width, x, 0);
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







