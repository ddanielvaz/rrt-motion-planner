#include "FierroControl.hh"
#include "SkidSteerControlBased.hh"

FierroControl::FierroControl(SkidSteerControlBased *robot)
{
    p3at = robot;
}

void FierroControl::InitializeControllerWeights(double w1, double w2, double w3,
                                           double w4)
{
    cout << "Initializing Controller Weights." << endl;
    k1 = w1;
    k2 = w2;
    k3 = w3;
    k4 = w4;
}

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, e utilizando uma lei de seguimento de trajetória encontra novos valores de torque que serão aplicados ao modelo dinâmico, gerando uma trajetória acompanhada por tal lei de controle.
*/
void FierroControl::run(const double *curr_st, const double *ref_st, double *u)
{
    double control_inputs[2];
    Pioneer3ATState curr_state(curr_st);
    Pioneer3ATState ref_state(ref_st);
    ErrorState e, de;
    VelocityState vc, dvc;
//     cout << "curr_state.x: " << curr_state.x << " curr_state.y: " << curr_state.y << " curr_state.psi: " << curr_state.psi << endl;
//     cout << "curr_state.v: " << curr_state.v << " curr_state.w: " << curr_state.w << endl;
//     cout << "ref_state.x: " << ref_state.x << " ref_state.y: " << ref_state.y << " ref_state.psi: " << ref_state.psi << endl;
//     cout << "ref_state.v: " << ref_state.v << " ref_state.w: " << ref_state.w << endl;

    ComputeErrorState(curr_state, ref_state, &e);
//     cout << "Error.x: " << e.e1 << " Error.y: " << e.e2 << " Error.psi: " << e.e3 << endl;

    ComputeDerivativeErrorState(curr_state, ref_state, e, &de);
//     cout << "DError.x: " << de.e1 << " DError.y: " << de.e2 << " DError.psi: " << de.e3 << endl;

    ComputeAuxVelocity(ref_state, e, &vc);
//     cout << "AuxV.v: " << vc.v << " AuxV.w: " << vc.w << endl;

    ComputeDerivativeAuxVelocity(ref_state, e, de, &dvc);
//     cout << "DAuxV.v: " << dvc.v << " DAuxV.w: " << dvc.w << endl;

    ComputeControlInput(vc, dvc, curr_state, control_inputs);
//     cout << "control_inputs[0]= " << control_inputs[0] << " control_inputs[1]= " << control_inputs[1] << endl;

    memcpy(u, control_inputs, sizeof(double) * 2);
}

void FierroControl::ComputeErrorState(Pioneer3ATState curr, Pioneer3ATState ref,
                                      ErrorState *error)
{
    double x_diff, y_diff;
    x_diff = ref.x - curr.x;
    y_diff = ref.y - curr.y;
    error->e1 = x_diff * cos(curr.psi) + y_diff * sin(curr.psi);
    error->e2 = -x_diff * sin(curr.psi) + y_diff * cos(curr.psi);
    error->e3 = ref.psi - curr.psi;
}

void FierroControl::ComputeDerivativeErrorState(Pioneer3ATState curr,
                                                Pioneer3ATState ref,
                                                ErrorState error,
                                                ErrorState *derror)
{
    derror->e1 = curr.w * error.e2 - curr.v + ref.v * cos(error.e3);
    derror->e2 = -curr.w * error.e1 + ref.v * sin(error.e3);
    derror->e3 = ref.w - curr.w;
}

void FierroControl::ComputeAuxVelocity(Pioneer3ATState ref, ErrorState error,
                                       VelocityState *vc)
{
    vc->v = ref.v * cos(error.e3) + k1 * error.e1;
    vc->w = ref.w + k2 * ref.v * error.e2 + k3 * ref.v * sin(error.e3);
}

void FierroControl::ComputeDerivativeAuxVelocity(Pioneer3ATState ref,
                                                 ErrorState error,
                                                 ErrorState derror,
                                                 VelocityState *dvc)
{
    dvc->v = k1 * derror.e1 - ref.v * sin(error.e3) * derror.e3;
    dvc->w = k2 * ref.v * derror.e2 + k3 * ref.v * cos(error.e3) * derror.e3;
}

void FierroControl::ComputeControlInput(VelocityState vc, VelocityState dvc,
                                        Pioneer3ATState curr, double *u)
{
    u[0] = dvc.v + k4 * (vc.v - curr.v);
    u[1] = dvc.w + k4 * (vc.w - curr.w);
}
