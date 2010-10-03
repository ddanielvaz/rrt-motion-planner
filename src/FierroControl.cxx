#include "FierroControl.hh"
#include "SkidSteerControlBased.hh"

FierroControl::FierroControl(SkidSteerControlBased *robot)
{
    p3at = robot;
}

void FierroControl::InitializeControllerWeights(double w1, double w2, double w3,
                                           double w4)
{
    k1 = w1;
    k2 = w2;
    k3 = w3;
    k4 = w4;
}

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, e utilizando uma lei de seguimento de trajetória encontra novos valores de torque que serão aplicados ao modelo dinâmico, gerando uma trajetória acompanhada por tal lei de controle.
*/
void FierroControl::run(const double *curr_st, const double *ref_st,
                        const double *ctl, double *tracked_st)
{
    Pioneer3ATState curr_state(curr_st);
    Pioneer3ATState ref_state(ref_st);
    ErrorState e, de;
    VelocityState vc, dvc, control;
    InitializeControllerWeights(0.5, 0.5, 0.5, 0.5);
    ComputeErrorState(curr_state, ref_state, &e);
    ComputeDerivativeErrorState(curr_state, ref_state, e, &de);
    ComputeAuxVelocity(ref_state, e, &vc);
    ComputeDerivativeAuxVelocity(ref_state, e, de, &dvc);
    ComputeControlInput(vc, dvc, curr_state, &control);
    cout << "curr_state.v: " << curr_state.v << " curr_state.w: " << curr_state.w << endl;
    cout << "ref_state.v: " << ref_state.v << " ref_state.w: " << ref_state.w << endl;
    cout << "linear accel: " << ctl[LINEAR_ACCEL]*DELTA_T << " angular accel: " << ctl[ANGULAR_ACCEL]*DELTA_T << endl;
    cout << "v_control= " << control.v << " w_control= " << control.w << endl << endl;
    memcpy(tracked_st, ref_st, sizeof(double) * 5);
}

void FierroControl::ComputeErrorState(Pioneer3ATState ref, Pioneer3ATState curr,
                                      ErrorState *error)
{
    double x_diff, y_diff;
    x_diff = ref.x - curr.x;
    y_diff = ref.y - curr.y;
    error->e1 = cos(curr.psi) * x_diff + sin(curr.psi) * y_diff;
    error->e2 = -sin(curr.psi) * x_diff + cos(curr.psi) * y_diff;
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
    vc->w = ref.w + k2 * ref.v * error.e2 + k3 *ref.v *sin(error.e3);
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
                                        Pioneer3ATState curr, VelocityState *u)
{
    u->v = dvc.v + k4 * (vc.v - curr.v);
    u->w = dvc.w + k4 * (vc.w - curr.w);
}
