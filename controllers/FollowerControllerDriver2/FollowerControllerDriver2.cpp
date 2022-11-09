/*
 * File:          FollowerControllerDriver.cpp
 * Date:          February 27, 2015
 * Description:   Follower vehicle controller 
 * Authors:       Oussama Karoui - www.oussamakaroui.com
 *                Emna Guerfala
 */
#include "PlatoonFollowerController.h"
// to be used as array indices
enum { X, Y, Z };


#define COMMUNICATION_CHANNEL 2
//Distance de reference
#define REFERENCE_DISTANCE 3
#define SAFETY_DISTANCE 7
#define INITIAL_SPEED 50.0

#define KP 1
#define KI 0.006
#define KD 2




// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  FollowerControllerDriver* controller = new FollowerControllerDriver();
  controller->run();
  delete controller;
  return 0;
}




