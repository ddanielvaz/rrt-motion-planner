#include "SpecificMeterP3AT.hh"

#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;

SpecificMeterP3AT::SpecificMeterP3AT(const double *gstate)
{
    goal_state = (double*)malloc(sizeof(double)*P3AT_N_STATES);
    if(!goal_state)
    {
        cerr << "Erro de alocação de memória. Constructor SpecificMeterP3AT" << endl;
        exit(1);
    }
    memcpy(goal_state, gstate, sizeof(double)*P3AT_N_STATES);
}

double SpecificMeterP3AT::DistanceWeight(const double *istate, const double *fstate)
{
    double dx, dy, angle, d, dv, dw;
    //distancia translacional
    dx = fstate[STATE_X] - istate[STATE_X];
    dy = fstate[STATE_Y] - istate[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = normalize_angle(fstate[STATE_THETA] - istate[STATE_THETA]);
    if(!memcmp(fstate, goal_state, sizeof(double)*P3AT_N_STATES))
    {
        if(d < DISTANCE_TOL)
        {
            dv = 2.0 * fabs(fstate[STATE_V] - istate[STATE_V]);
            dw = 2.0 * fabs(fstate[STATE_W] - istate[STATE_W]);
            return d + fabs(angle)/M_PI_2l + dv + dw;
        }
    }
    dv = fabs(fstate[STATE_V] - istate[STATE_V]);
    dw = fabs(fstate[STATE_W] - istate[STATE_W]);
    return d + fabs(angle)/M_PI_2l + dv + dw;
//     return d + fabs(angle)/M_PI_2l;
}

bool SpecificMeterP3AT::GoalStateAchieved(const double *state, const double *goal)
{
    double dx, dy, d, angle, dv, dw;
    //distancia translacional
    dx = goal[STATE_X] - state[STATE_X];
    dy = goal[STATE_Y] - state[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = fabs(normalize_angle(goal[STATE_THETA] - state[STATE_THETA]));
    dv = fabs(state[STATE_V] - goal[STATE_V]);
    dw = fabs(state[STATE_W] - goal[STATE_W]);
    if ( d <= GOAL_TRANSLATIONAL_TOLERANCE && angle <= MAX_ROTATIONAL_TOLERANCE &&
        dv <= MAX_LIN_SPEED_TOLERANCE && dw <= MAX_ROT_SPEED_TOLERANCE)
        return true;
    return false;
}
