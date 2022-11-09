/*
 * File:          driver.h
 * Date:          September 2014
 * Description:   Driver library to be used with the 'Car' proto (or any proto inherited by 'Car') and the car library
 * Author:        david.mansolino@epfl.ch
 * Modifications: 
 * Comments:      Sponsored by the CTI project RO2IVSim (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#ifndef DRIVER_H
#define DRIVER_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  OFF=0, 
  RIGHT,
  LEFT
} wbu_indicator_state;

void wbu_driver_init();
void wbu_driver_cleanup();
int wbu_driver_step();

// positive: turn right, negative: turn left
void wbu_driver_set_steering_angle(double steering_angle);
double wbu_driver_get_steering_angle();

void wbu_driver_set_cruising_speed(double speed);
double wbu_driver_get_target_cruising_speed();

double wbu_driver_get_current_speed();

void wbu_driver_set_throttle(double throttle);
double wbu_driver_get_throttle();

void wbu_driver_set_brake(double brake);
double wbu_driver_get_brake();

void wbu_driver_set_indicator(int state);
void wbu_driver_set_indicator_warning(bool state);

void wbu_driver_set_dipped_beams(bool state);
void wbu_driver_set_antifog_lights(bool state);

bool  wbu_driver_get_dipped_beams();
bool  wbu_driver_get_antifog_lights();

double wbu_driver_get_rpm();
int    wbu_driver_get_gear();
void   wbu_driver_set_gear(int gear);
int    wbu_driver_get_gear_number();

#ifdef __cplusplus
}
#endif

#endif // DRIVER_H
