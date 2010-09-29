#ifndef __MATH_FUNCTIONS_HH__
#define __MATH_FUNCTIONS_HH__

#include "Constants.hh"

#include <cmath>

double sgn(double x);

double limit_steering(double steering, double max_steering_angle);
double limit_speed(double speed, double max_speed);

double radians(const double d);
double degrees(const double r);
double normalize_angle(const double r);
double nearest_quantified_speed(double speed, double speed_step);

#endif
