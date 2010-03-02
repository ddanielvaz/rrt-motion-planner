#ifndef _UTILS_COMMOM_H
#define _UTILS_COMMOM_H

#include <math.h>
#include <iostream>
using namespace std;
#include <ctime>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

typedef boost::minstd_rand base_generator_type;
base_generator_type generator(time(NULL));
boost::uniform_real<> uni_dist(0,1);
boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);

#define GOAL_TRANSLATIONAL_TOLERANCE 0.3
//5 graus
#define ROTATIONAL_TOLERANCE 0.087
//10 graus
#define MAX_ROTATIONAL_TOLERANCE 3.15//0.1745
//mudanca maxima de angulo de estercamento em rad (3 graus)
#define MAX_STEER_DIFF 0.05235
//angulo maximo de estercamento em rad (30 graus)
#define MAX_STEERING 0.523598
//mudanca maxima de velocidade
#define MAX_SPEED_DIFF 0.3
//velocidade maxima
#define MAX_SPEED 1.0

#define DELTA_T 0.05
#define TIME_STEP 1.0

#define N_STATES 3
#define N_CONTROLS 11

enum
{
    STATE_X=0, STATE_Y, STATE_THETA
};

enum
{
    CONTROL_VELOCITY=0, CONTROL_STEERING_ANGLE
};

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
    double dx, dy, angle;
    //distancia translacional
    dx = state0[STATE_X] - state[STATE_X];
    dy = state0[STATE_Y] - state[STATE_Y];
    angle = normalize_angle(state0[STATE_THETA] - state[STATE_THETA]);
    //return sqrt(dx*dx + dy*dy) + sqrt(fabs(angle));
    return sqrt(dx*dx + dy*dy) + fabs(angle);
}

bool goal_state_reached(const double *state, const double *goal)
{
    double dx, dy, angle;
    //distancia translacional
    dx = goal[STATE_X] - state[STATE_X];
    dy = goal[STATE_Y] - state[STATE_Y];
    angle = normalize_angle(goal[STATE_THETA] - state[STATE_THETA]);
    if (sqrt(dx*dx + dy*dy) < GOAL_TRANSLATIONAL_TOLERANCE && 
         angle < MAX_ROTATIONAL_TOLERANCE)
        return true;
    return false;
}

void biased_sampling(const double *bounds, double *rand_state)
{
    double w=bounds[0], h=bounds[1], rand;
    rand = uni();
    //cout << "[1] RAND: " << rand;
    rand_state[STATE_X] = w*rand;
    rand = uni();
    //cout << " [2] RAND: " << rand << endl;
    rand_state[STATE_Y] = h*rand;
    rand_state[STATE_THETA] = (2.0 * rand - 1.0) * M_PI;
}

void initiate_rand_number_generator()
{
    generator.seed(time(NULL));
    for(int i = 0; i < 10; i++)
        uni();
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
