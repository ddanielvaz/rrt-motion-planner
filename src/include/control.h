#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <cmath>
#include <cstring>

#include "constants.h"

// Forward declaration - resolver problema de compilação de interdependência
// entre as classes TrackingControlPioneer3AT e SkidSteerControlBased
class SkidSteerControlBased;


class Pioneer3ATState
{
    public:
        Pioneer3ATState(void);
        Pioneer3ATState(const double *state);
        double x,y,psi,v,w;
};

Pioneer3ATState::Pioneer3ATState(void)
{
//     cout << "Warning Pioneer3ATState instance is uninitialized." << endl;
}

Pioneer3ATState::Pioneer3ATState(const double *state)
{
//     cout << "Initializing pioneer 3AT states - (x,y,psi,v,w)" << endl;
    x = state[STATE_X];
    y = state[STATE_Y];
    psi = state[STATE_THETA];
    v = state[STATE_V];
    w = state[STATE_W];
}

/**
Esta função auxiliar, implementa a transformação número (2) para estimar a
velocidade do robô P3AT.
--------------------------------------------------------------------------------
dq   => diferença entre estados (referência e atual)
eta  => vetor velocidade (linear e angular)
S(q) => Matriz que rege a cinemática do robô Pioneer3AT
--------------------------------------------------------------------------------
(1) dq = S(q) * eta
então:
(2) eta = S^-1(q) * dq

Obs: Como S(q) não é quadrada é calculada a pseudo-inversa.
*/
Pioneer3ATState calculate_speed(Pioneer3ATState dq, double theta)
{
    const double xcir=0.008, xcir2_1=(xcir*xcir)+1.0;
    Pioneer3ATState vel;
    vel.x = dq.x * cos(theta) + dq.y * sin(theta);
    vel.psi = dq.x*(-xcir*sin(theta)/xcir2_1) + dq.y*(xcir*cos(theta)/xcir2_1) + dq.psi/xcir2_1;
    return vel;
}

class TrackingControlPioneer3AT
{
    public:
        void run(const double *, const double *, const double *, double *);
        SkidSteerControlBased *p3at_robot;
    private:
        void ZDot2Flow(Pioneer3ATState, const double *, double *);
        void ZDotFlow(Pioneer3ATState, double *);
        void ZFlow(Pioneer3ATState, double *);
};

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, e utilizando uma lei de seguimento de trajetória encontra novos valores de torque que serão aplicados ao modelo dinâmico, gerando uma trajetória acompanhada por tal lei de controle.
*/
void TrackingControlPioneer3AT::run(const double *curr_st, const double *ref_st,
                          const double *ctl, double *tracked_st)
{
    Pioneer3ATState curr_state(curr_st);
    Pioneer3ATState ref_state(ref_st);
    double zd[2], z[2];
//     cout << "curr_state.x: " << curr_state.x << " curr_state.y: " << curr_state.y
//           << " curr_state.psi: " << curr_state.psi << " curr_state.v: " << curr_state.v
//           << " curr_state.w: " << curr_state.w << endl;
// 
//     cout << "ref_state.x: " << ref_state.x << " ref_state.y: " << ref_state.y
//           << " ref_state.psi: " << ref_state.psi << " ref_state.v: " << ref_state.v
//           << " ref_state.w: " << ref_state.w << endl;
// 
//     cout << "linear accel: " << ctl[LINEAR_ACCEL] << " angular accel: " << ctl[ANGULAR_ACCEL] << endl;
    memcpy(tracked_st, ref_st, sizeof(double) * 5);
}

/**
As equações que representam o campo vetorial da segunda derivada de z.
zdot2[0] = epslon*cos(psi) - eta1 * eta2 * sin(psi);
zdot2[1] = epslon*sin(psi) + eta1 * eta2 * cos(psi);
--------------------------------------------------------------------------------
(controle) epslon = aceleração linear
(espaço de estados) eta1 = velocidade linear
(espaço de estados) eta2 = velocidade angular
*/
void TrackingControlPioneer3AT::ZDot2Flow(Pioneer3ATState state, const double *u, double *zdot2)
{
    zdot2[0] = u[LINEAR_ACCEL]*cos(state.psi) - state.v * state.w * sin(state.psi);
    zdot2[1] = u[LINEAR_ACCEL]*sin(state.psi) + state.v * state.w * cos(state.psi);
}

/**
As equações que representam o campo vetorial da primeira derivada de z.
zdot[0] = eta1*cos(psi);
zdot[1] = eta1*sin(psi);
--------------------------------------------------------------------------------
(espaço de estados) eta1 = velocidade linear
*/
void TrackingControlPioneer3AT::ZDotFlow(Pioneer3ATState state, double *zdot)
{
    zdot[0] = state.v * cos(state.psi);
    zdot[1] = state.v * sin(state.psi);
}

/**
As equações que representam z.
z[0] = x - xcir*cos(psi);
z[1] = y - xcir*sin(psi);
*/
void TrackingControlPioneer3AT::ZFlow(Pioneer3ATState state, double *z)
{
    z[0] = state.x - p3at_robot->xcir * cos(state.psi);
    z[1] = state.y - p3at_robot->xcir * sin(state.psi);
}
#endif