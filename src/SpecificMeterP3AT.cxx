#include "SpecificMeterP3AT.hh"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <lemon/random.h>

using namespace std;
using namespace lemon;

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
    double dx, dy, angle, d, dv, dw, radius_to_qgoal;
    dv = dw = 0.0;
    // Se nó da árvore que será utilizado como início da expansão estiver dentro
    // de um certo raio do objetivo, levar em consideração velocidades ao
    // calcular peso da distância.
    dx = goal_state[STATE_X] - fstate[STATE_X];
    dy = goal_state[STATE_Y] - fstate[STATE_Y];
    radius_to_qgoal = sqrt(dx*dx + dy*dy);
    //distancia translacional
    dx = fstate[STATE_X] - istate[STATE_X];
    dy = fstate[STATE_Y] - istate[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    angle = normalize_angle(fstate[STATE_THETA] - istate[STATE_THETA]);
//     dv = fabs(fstate[STATE_V] - istate[STATE_V]);
//     dw = fabs(fstate[STATE_W] - istate[STATE_W]);
    if(radius_to_qgoal < 0.7)
    {
        dv = 0.8 * fabs(fstate[STATE_V] - istate[STATE_V]);
        dw = 0.8 * fabs(fstate[STATE_W] - istate[STATE_W]);
    }
    else
    {
        dv = 0.5 * fabs(fstate[STATE_V] - istate[STATE_V]);
        dw = 0.5 * fabs(fstate[STATE_W] - istate[STATE_W]);
    }
    return d + fabs(angle)/M_PI_2l + dv + dw;
}

double SpecificMeterP3AT::NearestNodeMetric(const double *istate, double *fstate)
{
    double dx, dy, angle, d, radius_to_qgoal, dv, dw;
    // Se nó da árvore que será utilizado como início da expansão estiver dentro
    // de um certo raio do objetivo, atualizar velocidades do estado aleatório,
    // fstate, para valores iguais ao do goal_state
    dx = goal_state[STATE_X] - istate[STATE_X];
    dy = goal_state[STATE_Y] - istate[STATE_Y];
    angle = fabs(normalize_angle(fstate[STATE_THETA] - istate[STATE_THETA]));
    radius_to_qgoal = sqrt(dx*dx + dy*dy) + angle/M_PI_2l;
    if(radius_to_qgoal < 0.30 && rnd() > 0.50)
    {
        fstate[STATE_V] = goal_state[STATE_V];
        fstate[STATE_W] = goal_state[STATE_W];
    }
    //distancia translacional
    dx = fstate[STATE_X] - istate[STATE_X];
    dy = fstate[STATE_Y] - istate[STATE_Y];
    d = sqrt(dx*dx + dy*dy);
    dv = 0.7 * fabs(fstate[STATE_V] - istate[STATE_V]);
    dw = 0.7 * fabs(fstate[STATE_W] - istate[STATE_W]);
    return d + angle/M_PI_2l + dv + dw;
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
