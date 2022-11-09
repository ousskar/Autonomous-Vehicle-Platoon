/*
 * File:          Traffic_light_1.c
 * Date:          July 25, 2013
 * Description:   Simple traffic light controller example
 * Author:        Guillaume Michel
 * Modifications: 
 */

#include <webots/robot.h>
#include <webots/led.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
  WbDeviceTag red_light,orange_light,green_light;
  red_light = wb_robot_get_device("red light");
  orange_light = wb_robot_get_device("orange light");
  green_light = wb_robot_get_device("green light");
  
  int t = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    t += TIME_STEP;
    // Turn on the red light.
    if (t==TIME_STEP)
      wb_led_set(red_light,1);
    // Turn off the red light and turn on the green light after 10 seconds.
    if (t==156*TIME_STEP) {
      wb_led_set(red_light,0);
      wb_led_set(green_light,1);
    }
    // Turn off the green light and turn on the orange light after 8 seconds.
    if (t==177*TIME_STEP) {
      wb_led_set(green_light,0);
      wb_led_set(orange_light,1);
    }
    // Turn off the orange light and restart after 2 seconds.
    if (t==302*TIME_STEP) {
      wb_led_set(orange_light,0);
      t=0;
    }
  };
  
  wb_robot_cleanup();
  
  return 0;
}
