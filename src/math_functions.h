#ifndef _MATH_FUNCTIONS_H_
#define _MATH_FUNCTIONS_H_

#include <math.h>

inline double limit_steering(double steering)
{
    return (fabs(steering) < MAX_STEERING) ? steering:MAX_STEERING;
}

inline double sgn(double x)
{
    if(x > 0.0)
        return 1.0;
    else if(x < 0.0)
        return -1.0;
    return 0.0;
}

inline double limit_speed(double speed, double max_speed)
{
    return (fabs(speed) < max_speed) ? speed:sgn(speed)*max_speed;
}

static inline
double radians(const double d) 
{
    return (M_PI * (d) / 180.0);
}

static inline
double degrees(const double r)
{
    return(180.0 * (r) / M_PI);
}

static inline
double normalize_angle(const double r)
{
    double pi2 = 2 * M_PI;
    return r - pi2 * floor((r + M_PI) / pi2);
}

#endif
