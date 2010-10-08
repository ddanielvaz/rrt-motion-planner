#include "DistanceMeter.hh"
#include "Constants.hh"

double DistanceMeter::DistanceWeight(const double *istate, const double *fstate)
{
    double dx, dy, angle, d;
    //distancia translacional
    dx = fstate[STATE_X] - istate[STATE_X];
    dy = fstate[STATE_Y] - istate[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = normalize_angle(fstate[STATE_THETA] - istate[STATE_THETA]);
    return d + fabs(angle)/M_PI_2l;
}

bool DistanceMeter::GoalStateAchieved(const double *state, const double *goal)
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

double DistanceMeter::Euclidean(const double *istate, const double *fstate)
{
    double dx, dy;
    //distancia euclideana
    dx = fstate[STATE_X] - istate[STATE_X];
    dy = fstate[STATE_Y] - istate[STATE_Y];
    return sqrt(dx*dx + dy*dy);
}
