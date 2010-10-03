#include "CaraccioloControl.hh"
#include "SkidSteerControlBased.hh"

CaraccioloControl::CaraccioloControl(SkidSteerControlBased *robot)
{
    p3at = robot;
}

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, e utilizando uma lei de seguimento de trajetória encontra novos valores de torque que serão aplicados ao modelo dinâmico, gerando uma trajetória acompanhada por tal lei de controle.
*/
void CaraccioloControl::run(const double *curr_st, const double *ref_st, const double *ctl, double *tracked_st)
{
    Pioneer3ATState curr_state(curr_st);
    Pioneer3ATState ref_state(ref_st);
//     double x_cir = p3at->GetXCIR();
    double z[2], dz[2], dz2[2];
    double z_d[2], dz_d[2], dz2_d[2];
    double z_diff[2], dz_diff[2], dz2_diff[2];
    double Ka[2], Kv[2], Kp[2];
    double r[2];
    double d_epslon, u1, u2;
    double zero_ctl[2];
    zero_ctl[0] = 0.0;
    zero_ctl[1] = 0.0;
    // Definindo valores de ganhos
    Ka[0] = Ka[1] = 1.0;
    Kv[0] = Kv[1] = 1.0;
    Kp[0] = Kp[1] = 1.0;
    // Mudança de variáveis no espaço de estados
    // Calculando novos valores para estado atual
    ZFlow(curr_state, z);
    ZDotFlow(curr_state, dz);
    // FIXME: utilizando entrada de controle = {0.0, 0.0}
    // Verificar se é isso mesmo que deve ser feito
    ZDot2Flow(curr_state, zero_ctl, dz2);
    // Calculando novos valores para estado desejado
    ZFlow(ref_state, z_d);
    ZDotFlow(ref_state, dz_d);
    ZDot2Flow(ref_state, ctl, dz2_d);
    // Calculando diferenca entre estado desejado e atual
    z_diff[0] = z_d[0] - z[0];
    z_diff[1] = z_d[1] - z[1];
    // Calculando diferenca entre estado desejado e atual ( primeira derivada )
    dz_diff[0] = dz_d[0] - dz[0];
    dz_diff[1] = dz_d[1] - dz[1];
    // Calculando diferenca entre estado desejado e atual ( segunda derivada )
    dz2_diff[0] = dz2_d[0] - dz2[0];
    dz2_diff[1] = dz2_d[1] - dz2[1];
    // Lei de controle
    r[0] = Ka[0] * dz2_diff[0] + Kv[0] * dz_diff[0] + Kp[0] * z_diff[0];
    r[1] = Ka[1] * dz2_diff[1] + Kv[1] * dz_diff[1] + Kp[1] * z_diff[1];
    // FIXME: Qual estado deve-se usar? O estado atual ou estado de referência???
    d_epslon = r[0] * cos(curr_state.psi) + (r[1] * sin(curr_state.psi) * curr_state.v * curr_state.w * curr_state.w);
    // u1 = integral(d_epslon) = epslon
    u1 = d_epslon * DELTA_T;
    u2 = (-r[0] * sin(curr_state.psi) + r[1] * cos(curr_state.psi) - 2 * u1 * curr_state.w)/curr_state.v;
//     cout << "z[0]: " << z[0] << " z[1]: " << z[1] << endl;
//     cout << "z_d[0]: " << z_d[0] << " z_d[1]: " << z_d[1] << endl;
//     cout << "dz[0]: " << dz[0] << " dz[1]: " << dz[1] << endl;
//     cout << "dz_d[0]: " << dz_d[0] << " dz_d[1]: " << dz_d[1] << endl;
//     cout << "dz2[0]: " << dz2[0] << " dz2[1]: " << dz2[1] << endl;
//     cout << "dz2_d[0]: " << dz2_d[0] << " dz2_d[1]: " << dz2_d[1] << endl;
//     cout << "z_diff[0]: " << z_diff[0] << " z_diff[1]: " << z_diff[1] << endl;
//     cout << "dz_diff[0]: " << dz_diff[0] << " dz_diff[1]: " << dz_diff[1] << endl;
//     cout << "dz2_diff[0]: " << dz2_diff[0] << " dz2_diff[1]: " << dz2_diff[1] << endl;
//     cout << "r[0]: " << r[0] << " r[1]: " << r[1] << endl;
//     cout << "old_u1: " << ctl[0]*DELTA_T << " old_u2: " << ctl[1]*DELTA_T << endl;
    cout << "linear accel: " << ctl[LINEAR_ACCEL] << " angular accel: " << ctl[ANGULAR_ACCEL] << endl;
    cout << "u1: " << u1 << " u2: " << u2 << endl;
//     cout << "curr_state.x: " << curr_state.x << " curr_state.y: " << curr_state.y
//           << " curr_state.psi: " << curr_state.psi << " curr_state.v: " << curr_state.v
//           << " curr_state.w: " << curr_state.w << endl;
// 
//     cout << "ref_state.x: " << ref_state.x << " ref_state.y: " << ref_state.y
//           << " ref_state.psi: " << ref_state.psi << " ref_state.v: " << ref_state.v
//           << " ref_state.w: " << ref_state.w << endl;
// 
    cout << endl;
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
void CaraccioloControl::ZDot2Flow(Pioneer3ATState state, const double *u, double *zdot2)
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
void CaraccioloControl::ZDotFlow(Pioneer3ATState state, double *zdot)
{
    zdot[0] = state.v * cos(state.psi);
    zdot[1] = state.v * sin(state.psi);
}

/**
As equações que representam z.
z[0] = x - xcir*cos(psi);
z[1] = y - xcir*sin(psi);
*/
void CaraccioloControl::ZFlow(Pioneer3ATState state, double *z)
{
    double x_cir = p3at->GetXCIR();
    z[0] = state.x - x_cir * cos(state.psi);
    z[1] = state.y - x_cir * sin(state.psi);
}
