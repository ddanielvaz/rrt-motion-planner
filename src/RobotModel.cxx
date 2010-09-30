#include "RobotModel.hh"

void RobotModel::dflow(const double *x, const double *u, double *dx)
{
    cout << "VIRTUAL METHOD dflow: Warning overwrite it in child class." << endl;
}

void RobotModel::EstimateNewState(const double *x,
                                          const double *u, double *dx)
{
    cout << "VIRTUAL METHOD EstimateNewState: Warning overwrite it in child class." << endl;
}

void RobotModel::GenerateInputs(void)
{
    cout << "VIRTUAL METHOD GenerateInputs: Warning overwrite it in child class." << endl;
}

void RobotModel::GetValidInputs(const double *x)
{
    cout << "VIRTUAL METHOD GetValidInputs: Warning overwrite it in child class." << endl;
}
