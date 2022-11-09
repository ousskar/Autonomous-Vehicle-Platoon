#ifndef CONTROLLERFUNCTIONS_H_
#define CONTROLLERFUNCTIONS_H_
#include <ctime>

double PIDsetSafeDistance(double distance,double safeDistance,double speed1,double speed2,double currentSpeed);
double PIDsetSteer(double setpoint,double vehicle_angle);
double timer(clock_t start);
#endif /* CONTROLLERFUNCTIONS_H_ */
