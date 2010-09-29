#ifndef _MATH_FUNCTIONS_H_
#define _MATH_FUNCTIONS_H_

#include <cmath>

#include "constants.h"

inline double sgn(double x)
{
    if(x > 0.0)
        return 1.0;
    else if(x < 0.0)
        return -1.0;
    return 0.0;
}

inline double limit_steering(double steering, double max_steering_angle)
{
    return (fabs(steering) < max_steering_angle) ? steering:sgn(steering)*max_steering_angle;
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

static inline
double nearest_quantified_speed(double speed, double speed_step)
{
    double near_speed=0.0, temp, aux=1e7, i;
    for(i=-MAX_LIN_SPEED; i<=MAX_LIN_SPEED; i+=speed_step)
    {
        temp = i-speed;
        if(fabs(temp) < fabs(aux))
        {
            aux = temp;
            near_speed = i;
        }
    }
    return near_speed;
}

#endif
