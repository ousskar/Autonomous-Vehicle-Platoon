#include "ControllerFunctions.h"
#include <cmath>
#include <string>
#include <iostream>
#include <ctime>
// All the webots classes are defined in the "webots" namespace
using namespace std;


/* PID Control Safe Distance Gains */
extern double KPs;
extern double KIs;  //0.08
extern double KDs;


/* PID Control Steering Gains */
extern double KPstr;
extern double KIstr;  //0.08
extern double KDstr;



 /*
 *  PID Control : Maintain Safe Distance Between Vehicles
 */
double PIDsetSafeDistance(double distance,double safeDistance,double speed1,double speed2,double currentSpeed)
{
  double Vfollowed=0.0;
  if(speed1==speed2){
    Vfollowed=speed1;
  }else Vfollowed=currentSpeed;
  if(Vfollowed<80){
    KPs=2.0;
    KIs=0.2;
   }
  static double oldValue = 0.0;
  static double integral = 0.0;
  double result=0.0;
  
  double err = distance - safeDistance;
  double diff= err-oldValue;
  if(floor(abs(err))==0){
    integral=0.0;
    result= Vfollowed;
    return result;
    }
  if (integral < 10)
    integral += err;
  oldValue = err;
  result= ((KPs * err) + Vfollowed) + (KIs * integral) + (KDs * diff);
  return result;
}


/*
* PID Control : to steer the vehicle
*/

double PIDsetSteer(double setpoint,double vehicle_angle)
{
  double rs = 0.0; // the output of PID
  static double oldValue = 0.0;
  static double integral = 0.0;
  if ((signbit(setpoint) != signbit(vehicle_angle)))
  {
  vehicle_angle += 2*M_PI;
  } 
  double error = setpoint - vehicle_angle; 
  if (integral < 30 && integral > -30)
  integral += error;
  oldValue = error;
  double diff = error - oldValue;
  rs = KPstr * error +  KIstr * integral + KDstr * diff;
  return rs;
}

/*
*   Timer to fix the next sending time
*   @ return period of time
*/
double timer(clock_t start){
  double duration;
  start = clock();
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  return duration;
}
