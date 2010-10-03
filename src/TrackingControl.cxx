#include "TrackingControl.hh"

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, e utilizando uma lei de seguimento de trajetória encontra novos valores de torque que serão aplicados ao modelo dinâmico, gerando uma trajetória acompanhada por tal lei de controle.
*/
void TrackingControl::run(const double *curr_st, const double *ref_st, const double *ctl, double *tracked_st)
{
    cout << "VIRTUAL METHOD run: Warning overwrite it in child class." << endl;
}