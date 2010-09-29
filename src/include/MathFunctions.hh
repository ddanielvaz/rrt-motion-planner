#ifndef __MATH_FUNCTIONS_HH__
#define __MATH_FUNCTIONS_HH__

#include "Constants.hh"

#include <cmath>

inline double sgn(double x);

inline double limit_steering(double steering, double max_steering_angle);
inline double limit_speed(double speed, double max_speed);

inline double radians(const double d);
inline double degrees(const double r);
inline double normalize_angle(const double r);
inline double nearest_quantified_speed(double speed, double speed_step);

#endif
