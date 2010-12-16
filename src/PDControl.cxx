#include "PDControl.hh"
#include "SkidSteerControlBased.hh"

PDControl::PDControl(SkidSteerControlBased *robot)
{
    p3at = robot;
}

void PDControl::InitializeControllerWeights(double kpv, double kdv, double kpw,
                                           double kdw)
{
    cout << "Initializing Controller Weights." << endl;
    kp_v = kpv;
    kd_v = kdv;
    kp_w = kpw;
    kd_w = kdw;
}

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, e utilizando uma lei de seguimento de trajetória encontra novos valores de torque que serão aplicados ao modelo dinâmico, gerando uma trajetória acompanhada por tal lei de controle.
*/
void PDControl::run(const double *curr_st, const double *ref_st, double *u)
{
    double control_inputs[2];
    Pioneer3ATState curr_state(curr_st);
    Pioneer3ATState ref_state(ref_st);
    VelocityState prop_e, deriv_e;
//     cout << "#### STARTS ####" << endl;
//     cout << "curr_state.x: " << curr_state.x << " curr_state.y: " << curr_state.y << " curr_state.psi: " << curr_state.psi << endl;
//     cout << "ref_state.x: " << ref_state.x << " ref_state.y: " << ref_state.y << " ref_state.psi: " << ref_state.psi << endl;
//     cout << "curr_state.v: " << curr_state.v << " curr_state.w: " << curr_state.w << endl;
//     cout << "ref_state.v: " << ref_state.v << " ref_state.w: " << ref_state.w << endl;

    ComputeErrorState(curr_state, ref_state, &prop_e);
//     cout << "v_error: " << vc.v << " w_error: " << vc.w << endl;
    ComputeDerivativeErrorState(curr_state, ref_state, &deriv_e);
//     cout << "dv_error: " << dvc.v << " dw_error: " << dvc.w << endl;
    ComputeControlInput(prop_e, deriv_e, control_inputs);
//     cout << "control_inputs[0]= " << control_inputs[0] << " control_inputs[1]= " << control_inputs[1] << endl;
    memcpy(u, control_inputs, sizeof(double) * 2);
}

void PDControl::ComputeErrorState(Pioneer3ATState curr, Pioneer3ATState ref,
                                  VelocityState *verror)
{
    double x_diff, y_diff, psi_diff, xcir2_1, xcir;
    xcir = p3at->GetXCIR();
    xcir2_1 = (xcir * xcir) + 1.0;
    x_diff = ref.x - curr.x;
    y_diff = ref.y - curr.y;
    psi_diff = normalize_angle(ref.psi - curr.psi);
    verror->v = x_diff * cos(curr.psi) + y_diff * sin(curr.psi);
    verror->w = x_diff*(-xcir*sin(curr.psi)/xcir2_1) +
                y_diff*(xcir*cos(curr.psi)/xcir2_1) + psi_diff/xcir2_1;
}

void PDControl::ComputeDerivativeErrorState(Pioneer3ATState curr,
                                            Pioneer3ATState ref,
                                            VelocityState *verror)
{
    verror->v = (ref.v - curr.v)/DELTA_T;
    verror->w = (ref.w - curr.w)/DELTA_T;
}

void PDControl::ComputeControlInput(VelocityState prop_error, VelocityState deriv_error,
                                    double *u)
{
    u[0] = kp_v * prop_error.v + kd_v * deriv_error.v;
    u[1] = kp_w * prop_error.w + kd_w * deriv_error.w;
}
