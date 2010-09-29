#include "SkidSteerModel.hh"

SkidSteerModel::SkidSteerModel(int n_st, double x_cir)
{
    cout << "Criando instancia da classe SkidSteerModel." << endl;
    n_states = n_st;
    xcir = x_cir;
    double u[42][2]={{0.30, 0.00}, {0.30, 0.02}, {0.30, 0.07}, {0.30, 0.12},
                     {0.30, 0.17}, {0.30, 0.23}, {0.30, 0.28}, {0.30, 0.33},
                     {0.30, 0.38}, {0.30, 0.44}, {0.30, 0.49}, {-0.30, 0.00},
                     {-0.30, 0.02}, {-0.30, 0.07}, {-0.30, 0.12}, {-0.30, 0.17},
                     {-0.30, 0.23}, {-0.30, 0.28}, {-0.30, 0.33}, {-0.30, 0.38},
                     {-0.30, 0.44}, {-0.30, 0.49}, {0.30, -0.02}, {0.30, -0.07},
                     {0.30, -0.12}, {0.30, -0.17}, {0.30, -0.23}, {0.30, -0.28},
                     {0.30, -0.33}, {0.30, -0.38}, {0.30, -0.44}, {0.30, -0.49},
                     {-0.30, -0.02}, {-0.30, -0.07}, {-0.30, -0.12},
                     {-0.30, -0.17}, {-0.30, -0.23}, {-0.30, -0.28},
                     {-0.30, -0.33}, {-0.30, -0.38}, {-0.30, -0.44},
                     {-0.30, -0.49} };
    control_input temp[42];
    memcpy(temp, u, sizeof(double) * 2 * 42);
    for (int i=0; i<42; i++)
        inputs.push_back(temp[i]);
}

void SkidSteerModel::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[VX_SPEED] * cos(x[STATE_THETA]) - xcir * sin(x[STATE_THETA]) * u[ANGULAR_SPEED];
    dx[STATE_Y] = u[VX_SPEED] * sin(x[STATE_THETA]) + xcir * cos(x[STATE_THETA]) * u[ANGULAR_SPEED];
    dx[STATE_THETA] = u[ANGULAR_SPEED];
}

void SkidSteerModel::EstimateNewState(const double *x,
                                      const double *u, double *dx)
{
    double w1[3], w2[3], w3[3], w4[3], wtemp[3];
    int i;

    dflow(x, u, w1);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * DELTA_T * w1[i];
    dflow(wtemp, u, w2);

    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * DELTA_T * w2[i];
    dflow(wtemp, u, w3);

    for(i=0;i<3;i++)
        wtemp[i] = x[i] + DELTA_T * w3[i];
    dflow(wtemp, u, w4);

    for(i=0; i<3; i++)
        dx[i] = x[i] + (DELTA_T/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
}

SkidSteerModel::~SkidSteerModel()
{
    cout << "Destruindo instancia da classe SkidSteerModel." << endl;
}
