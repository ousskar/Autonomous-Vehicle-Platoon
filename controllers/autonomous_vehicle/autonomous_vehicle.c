/*
 * File:          autonomous_vehicle.c
 * Date:          January 4, 2010
 * Description:   Autonoumous vehicle controller example
 * Authors:       Yvan Bourquin - www.cyberbotics.com
 *                Olivier Michel - www.cyberbotics.com
 * Modifications: October 28, 2014 by David Mansolino
 */

#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/driver.h>
#include <webots/display.h>
#include <webots/emitter.h>
#include <webots/compass.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99
#define COMMUNICATION_CHANNEL 1
// Line following PID
#define KP 1
#define KI 0.006
#define KD 2

// Size of the yellow line angle filter
#define FILTER_SIZE 3


// Distance needed to send data for one time  
#define REFERENCE_DISTANCE 3
#define INITIAL_SPEED 50.0


// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;


WbDeviceTag compass;

//emitter
WbDeviceTag  communication;

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

// speedometer
WbDeviceTag display;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

/*
*    Platoon Behavior
*    0 - Normal status
*    * - Emergency status dependant to emergency codes
*/
double status=0;



//Enable Scenarios
bool enable_full_brake=true;






// Emergency variables
bool FB_em=false;  // Full brake emergency code 000001



//emergency codes
double FB_code = 1;





//define initial positions used for sending data in every Distance_Refernce 
 double x_initial =0.0;
 double y_initial =0.0;
 bool start_iteration=true;

void reference_distance_reached(double x,double y);
void call_emergency(int code);

void print_help() {
  printf("You can drive this car!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer\n");
  printf("[UP]/[DOWN] - accelerate/slow down\n");
}

void set_autodrive(bool onoff) {
  if (autodrive == onoff) return;
  autodrive = onoff;
  switch (autodrive) {
  case false:
    printf("switching to manual drive...\n");
    printf("hit [A] to return to auto-drive.\n");
    break;
  case true:
    printf("switching to auto-drive...\n");
    break;
  }
}

// set target speed
void set_speed(double kmh) {
  
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

void change_manual_steer_angle(int inc) {
  set_autodrive(false);
  
  double new_manual_steering = manual_steering + inc;
  if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0) {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering * 0.02);
  }

  if (manual_steering == 0)
    printf("going straight\n");
  else
    printf("turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right");
}

void check_keyboard() {
  int key = wb_robot_keyboard_get_key();
  switch (key) {
    case WB_ROBOT_KEYBOARD_UP:
      set_speed(speed + 5.0);
      break;
    case WB_ROBOT_KEYBOARD_DOWN:
      set_speed(speed - 5.0);
      break;
    case WB_ROBOT_KEYBOARD_RIGHT:
      change_manual_steer_angle(+1);
      break;
    case WB_ROBOT_KEYBOARD_LEFT:
      change_manual_steer_angle(-1);
      break;
    case 'A':
      set_autodrive(true);
      break;
  }
}

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = { 48, 119, 132 }; // road yellow (BGR format)
  int sumx = 0;  // summed x position of pixels
  int pixel_count = 0;  // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++; // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
       return UNKNOWN;
  
  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) { // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else { // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }
  
  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
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
    if (range < 12.0) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }
  
  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;
  
  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

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

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  double vel[3] = { coords[X] - gps_coords[X], coords[Y] - gps_coords[Y], coords[Z] - gps_coords[Z] };
  double dist = sqrt(vel[X] * vel[X] + vel[Y] * vel[Y] + vel[Z] * vel[Z]);
  
  // store into global variables
  gps_speed = dist / TIME_STEP * 3600.0;
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

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

int main(int argc, char **argv)
{ int channel;

      
  
  wbu_driver_init();
 // clock_t start = clock(), diff;
  //printf("The leading vehicle started at %d \n",start);
//emit data 

 communication = wb_robot_get_device("emitter");
 channel = wb_emitter_get_channel(communication);
    if (channel != COMMUNICATION_CHANNEL) {
      wb_emitter_set_channel(communication, COMMUNICATION_CHANNEL);
    }
  // check if there is a SICK and a display
  int j = 0;
  for (j=0; j<wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char * name = wb_device_get_name(device);
    if (strcmp(name,"lms291") == 0)
      enable_collision_avoidance = true;
    else if (strcmp(name,"display") == 0)
      enable_display = true;
  }
compass = wb_robot_get_device("compass");
wb_compass_enable (compass, TIME_STEP);
  // camera device
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  camera_fov = wb_camera_get_fov(camera);

  // SICK sensor
  if (enable_collision_avoidance) {
    sick = wb_robot_get_device("lms291");
    wb_camera_enable(sick, TIME_STEP);
    sick_width = wb_camera_get_width(sick);
    sick_range = wb_camera_get_max_range(sick);
    sick_fov = wb_camera_get_fov(sick);
  }
  
  // initialize gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
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

  print_help();
  
  // allow to switch to manual control
  wb_robot_keyboard_enable(TIME_STEP);
 
  // main loop
  while (wbu_driver_step() != -1) {
    // get user input
    check_keyboard();
   
    static int i = 0;
    
    // updates sensors only every TIME_STEP milliseconds
    if(i%(int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
    /*
    double current_speed =wbu_driver_get_current_speed();
    //printf("steering_angle=%lf\n",steering_angle);
    
    
    if(((steering_angle<-0.04) ||(steering_angle>0.4)) && current_speed>=50){
    set_speed(60);
    wbu_driver_set_brake(0.4);
    }
      */  
    
    /*
    else if(current_speed <50) set_speed(80); 
    printf("current_speed=%lf\n",current_speed);
    */
    
     if(start_iteration){
            const double *pos1 = wb_gps_get_values(gps);
              
             x_initial = pos1[0];
             y_initial = pos1[2]; 
            //printf("x = %lf \n",x);
   
          }
     const double *pos2 = wb_gps_get_values(gps);
     double x2,y2; 
     x2 = pos2[0];
     y2 = pos2[2]; 
     reference_distance_reached(x2,y2);  // Now we difuse data to the follower
   
   /*
   if(enable_full_brake){
    diff = clock() - start;
    int msec = diff * 1000 / CLOCKS_PER_SEC;
    printf("msec=%lf\n",msec);
    if((msec/1000)==14){
  
      call_emergency(1);
      
    }
   
   }
   */
   /*
   diff = clock() - start;

   int msec = diff * 1000 / CLOCKS_PER_SEC;
   if((msec%1000)==225){
   //set_speed(80);
   }
   
   printf("Time taken %d seconds %d milliseconds\n", msec/1000, msec%1000); 

    */


    
    /*
    //cruising_speed = wbu_driver_get_target_cruising_speed();
    printf("+++++++ Cruising Speed = %lf \n ",speed);
    
    double current_speed = 0.0;
    current_speed = wbu_driver_get_current_speed();
    printf("+++++++ Current Speed = %lf\n ",current_speed);
    

    printf("steering_angle = %lf \n",steering_angle);
  */
    
    
    
    
      // read sensors
      const unsigned char *camera_image = wb_camera_get_image(camera);
      const float *sick_data = NULL;
      if (enable_collision_avoidance)
        sick_data = wb_camera_get_range_image(sick);

      if (autodrive) {
      
        double yellow_line_angle = filter_angle(process_camera_image(camera_image));
        double obstacle_dist;
        double obstacle_angle;
        if (enable_collision_avoidance)
          obstacle_angle = process_sick_data(sick_data, &obstacle_dist);
        
        // avoid obstacles and follow yellow line
        if (enable_collision_avoidance && obstacle_angle != UNKNOWN)
        {
        
          wbu_driver_set_brake(0.0);
          double steer = steering_angle;
          if (obstacle_angle > 0.0 && obstacle_angle < 0.5)
            steer = steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
          else if (obstacle_angle > -0.5)
            steer = steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
          set_steering_angle(steer);
        } else
    if (yellow_line_angle != UNKNOWN) {
          wbu_driver_set_brake(0.0);
          set_steering_angle(applyPID(yellow_line_angle));
           }
           
           else  // we lost the line => brake
          wbu_driver_set_brake(0.4);
          
           
      }
      /*
     const double *north = wb_compass_get_values(compass);

     double angle = atan2(north[0],north[2] );
 
     double array[7] = {wb_gps_get_values(gps)[X],wb_gps_get_values(gps)[Y],wb_gps_get_values(gps)[Z],angle, gps_speed,steering_angle,speed};
      wb_emitter_send(communication, array, 7* sizeof(double));
        */   
    
    //const char *message = "Hello !";
  //   wb_emitter_send(communication, message, strlen(message) + 1);
     
      // update stuff
      compute_gps_speed();
      if (enable_display)
        update_display();

      start_iteration =false;    // wait for the next interation    
        
    }
    
    ++i;
 
  
  }

  wbu_driver_cleanup();

  return 0;  // ignored

}


void reference_distance_reached(double x,double y){
  double dis=0.0;
  double steer =wbu_driver_get_steering_angle();
  double current_speed=wbu_driver_get_current_speed();
  dis=sqrt(pow(x-x_initial,2)+pow(y-y_initial,2));
  const double *north = wb_compass_get_values(compass);
  double angle = atan2(north[0],north[2] );
  //printf("angle=%lf\n",angle);
  //printf("north0=%lf,north2=%lf\n",north[0],north[2]);
   
  if(FB_em){
     double array[7] = {wb_gps_get_values(gps)[X],wb_gps_get_values(gps)[Z],angle,steer,speed,current_speed,status};
     wb_emitter_send(communication, array, 7* sizeof(double));
  }else if(dis >= REFERENCE_DISTANCE){
     double array[7] = {wb_gps_get_values(gps)[X],wb_gps_get_values(gps)[Z],angle,steer,speed,current_speed,0};
     wb_emitter_send(communication, array, 7* sizeof(double));
   x_initial = wb_gps_get_values(gps)[X];
   y_initial = wb_gps_get_values(gps)[Z];  
   start_iteration=true;
  }
}

void call_emergency(int code)
{
  switch(code){
    case 1:

      set_speed(0);
      wbu_driver_set_brake(0.4);
      FB_em=true;   // Activate the full brake
      status=1;
      break;
  }
}