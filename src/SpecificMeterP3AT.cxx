#include "SpecificMeterP3AT.hh"

#include <iostream>
using namespace std;

double SpecificMeterP3AT::DistanceWeight(const double *istate, const double *fstate)
{
    double dx, dy, angle, d, dv, dw;
    //distancia translacional
    dx = fstate[STATE_X] - istate[STATE_X];
    dy = fstate[STATE_Y] - istate[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = normalize_angle(fstate[STATE_THETA] - istate[STATE_THETA]);
    dv = fabs(istate[STATE_V] - fstate[STATE_V]);
    dw = fabs(istate[STATE_W] - fstate[STATE_W]);
    return d + fabs(angle)/M_PI_2l + dv + dw;
}

bool SpecificMeterP3AT::GoalStateAchieved(const double *state, const double *goal)
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
