/*
 * File:          defective_street_light.c
 * Date:          October 29, 2014
 * Description:   Simple controller simulating a defective street light
 * Author:        David Mansolino
 * Modifications: 
 */
 
#include <webots/robot.h>
#include <webots/led.h>

#define TIME_STEP 50

int main(int argc, char **argv) {

  wb_robot_init();
  
  WbDeviceTag led = wb_robot_get_device("led");
  int step_to_perform = 0;
  
  while (wb_robot_step(TIME_STEP) != -1) {
    if (step_to_perform <= 0) {
      int led_state = wb_led_get(led);
      if(led_state == 255) {
        // led is on, switch it off for 20steps (1s)
        step_to_perform = 20;
        wb_led_set(led, 0);
      } else if (led_state == 170) {
        // led is switching on (2/3 power) switch it fully on for 60steps (3s)
        step_to_perform = 60;
        wb_led_set(led, 255);
      } else if (led_state == 85) {
        // led is switching on (1/3 power) continue switch on for 2steps (0.1s)
        step_to_perform = 2;
        wb_led_set(led, 170);
      } else {
        // led is off, try to switch on (1/3 power) for 2steps (0.1s)
        step_to_perform = 2;
        wb_led_set(led, 85);
      }
    } else
      step_to_perform--;
  };
  
  wb_robot_cleanup();
  
  return 0;
}
