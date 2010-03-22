#ifndef _UTILS_COMMOM_H_
#define _UTILS_COMMOM_H_

#include <math.h>
#include <iostream>
#include <ctime>

#include <cv.h>
#include <lemon/random.h>

using namespace std;
using namespace lemon;

#define GOAL_TRANSLATIONAL_TOLERANCE 0.5
//5 graus
#define ROTATIONAL_TOLERANCE 0.087
//10 graus
#define MAX_ROTATIONAL_TOLERANCE 2 * ROTATIONAL_TOLERANCE
//mudanca maxima de angulo de estercamento em rad (3 graus)
#define MAX_STEER_DIFF 0.05235
//angulo maximo de estercamento em rad (30 graus)
#define MAX_STEERING 0.523598
//mudanca maxima de velocidade
#define MAX_SPEED_DIFF 0.3
//velocidade maxima
#define MAX_SPEED 1.0

#define DELTA_T 0.05
#define INTEGRATION_TIME 1.0

#define GOAL_BIAS 0.15

#define N_STATES 3
#define N_CONTROLS 11

#define SCALE_FACTOR 20.0

enum
{
    STATE_X=0, STATE_Y, STATE_THETA
};

enum
{
    CONTROL_VELOCITY=0, CONTROL_STEERING_ANGLE
};

enum
{
    c_red=0, c_green, c_blue, c_yellow, c_light_green, c_black, c_white
};

CvScalar colors[] = {CV_RGB(1,0,0), CV_RGB(0,0.392,0), CV_RGB(0,0,1),
                     CV_RGB(1,1,0), CV_RGB(0.596,0.984,0.596), CV_RGB(0,0,0),
                     CV_RGB(1,1,1) };

const double
control_velocities[N_CONTROLS] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

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

double metric(const double *state0, const double *state)
{
    double dx, dy, angle, d;
    //distancia translacional
    dx = state0[STATE_X] - state[STATE_X];
    dy = state0[STATE_Y] - state[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = normalize_angle(state0[STATE_THETA] - state[STATE_THETA]);
    return d + fabs(angle)*(1.5/(d+0.001));
    //return d + fabs(angle)*1.5;
    //return d + fabs(angle)*(1.0 + d/3.0);
}

bool goal_state_reached(const double *state, const double *goal)
{
    double dx, dy, d, angle;
    //distancia translacional
    dx = goal[STATE_X] - state[STATE_X];
    dy = goal[STATE_Y] - state[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = fabs(normalize_angle(goal[STATE_THETA] - state[STATE_THETA]));
    if ( d < GOAL_TRANSLATIONAL_TOLERANCE && angle < MAX_ROTATIONAL_TOLERANCE)
        return true;
    return false;
}

void biased_sampling(const double *bounds, double *rand_state)
{
    double w=bounds[0], h=bounds[1];
    rand_state[STATE_X] = rnd(0.0, w);
    rand_state[STATE_Y] = rnd(0.0, h);
    rand_state[STATE_THETA] = rnd(-M_PI, M_PI);
}

void initiate_rand_number_generator()
{
    rnd.seedFromFile();
    return;
}

inline double limit_steering(double steering)
{
    return (fabs(steering) < MAX_STEERING) ? steering:MAX_STEERING;
}

inline double limit_speed(double speed)
{
    return (fabs(speed) < MAX_SPEED) ? speed:MAX_SPEED;
}

#endif
