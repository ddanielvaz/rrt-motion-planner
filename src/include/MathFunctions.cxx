#include "MathFunctions.hh"

double sgn(double x)
{
    if(x > 0.0)
        return 1.0;
    else if(x < 0.0)
        return -1.0;
    return 0.0;
}

double limit_steering(double steering, double max_steering_angle)
{
    return (fabs(steering) < max_steering_angle) ? steering:sgn(steering)*max_steering_angle;
}

double limit_speed(double speed, double max_speed)
{
    return (fabs(speed) < max_speed) ? speed:sgn(speed)*max_speed;
}

double radians(const double d) 
{
    return (M_PI * (d) / 180.0);
}

double degrees(const double r)
{
    return(180.0 * (r) / M_PI);
}

double normalize_angle(const double r)
{
    double pi2 = 2 * M_PI;
    return r - pi2 * floor((r + M_PI) / pi2);
}

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
