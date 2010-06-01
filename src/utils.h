#ifndef _UTILS_COMMOM_H_
#define _UTILS_COMMOM_H_

#include <math.h>
#include <iostream>
#include <ctime>

#include <lemon/random.h>

#include "constants.h"
#include "math_functions.h"

using namespace std;
using namespace lemon;

double metric(const double *state0, const double *state)
{
    double dx, dy, angle, d;
    //distancia translacional
    dx = state0[STATE_X] - state[STATE_X];
    dy = state0[STATE_Y] - state[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = normalize_angle(state0[STATE_THETA] - state[STATE_THETA]);
    if (d < 1.5)
        return d + fabs(angle)*0.25;
    else
        return d + fabs(angle)*2;
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

#endif
