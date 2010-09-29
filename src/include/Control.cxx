#include "Control.hh"

/* HACK: o que deveria ser feito é realizar uma dependência circular entre
as classes TrackingControlPioneer3AT e SkidSteerControlBased, então a classe
TrackingControlPioneer3AT teria acesso aos parametros do modelo do robô
representado pela classe SkidSteerControlBased e a classe SkidSteerControlBased
teria acesso ao método de acompanhamento de trajetória implementado pela classe
TrackingControlPioneer3AT.
*/
TrackingControlPioneer3AT::TrackingControlPioneer3AT(double x_cir)
{
    xcir = x_cir;
}

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
    z[0] = state.x - xcir * cos(state.psi);
    z[1] = state.y - xcir * sin(state.psi);
}
