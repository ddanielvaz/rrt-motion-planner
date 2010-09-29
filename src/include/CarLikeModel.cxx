#include "CarLikeModel.hh"

// #include <iostream>
// #include <vector>

// using namespace std;

CarLikeModel::CarLikeModel(double body_length, double *constraints, int n_st)
{
    cout << "Criando instancia da classe CarLikeModel" << endl;
    m_one_over_bodyLength = 1.0/body_length;
    max_v = constraints[0];
    max_steering_angle = constraints[1];
    n_states = n_st;
}

CarLikeModel::~CarLikeModel()
{
    cout << "Destruindo instancia da classe CarLikeModel" << endl;
}

void CarLikeModel::GenerateInputs(char *filename)
{
    ifstream accel_inputs(filename);
    char buf[1024], *pt, *acc, *nxt, *dv, *dw;
    control_input temp;
    int i, n_inputs;
    all_inputs.clear();
    while(accel_inputs.getline(buf, 1024))
    {
        n_inputs = atoi(buf);
        accel_inputs.getline(buf, 1024);    
        pt = buf;
        for(i=0;i<n_inputs;i++)
        {
            acc = strtok_r(pt, ";", &nxt);
            dv = strtok_r(acc, ",", &dw);
            temp.ctrl[LINEAR_ACCEL] = atof(dv);
            temp.ctrl[ANGULAR_ACCEL] = atof(dw);
            cout << "Linear acceleration: " << temp.ctrl[LINEAR_ACCEL]
                 << " Angular acceleration: " << temp.ctrl[ANGULAR_ACCEL] << endl;
            all_inputs.push_back(temp);
            pt = nxt;
        }
    }
}

void CarLikeModel::GetValidInputs(const double *x)
{
//     cout << "GET VALID INPUTS" << endl << endl ;
    double v, steering_angle;
    vector<control_input>::iterator it;
    inputs.clear();
//     cout << "curr_v: " << x[STATE_V] << " curr_w: " << x[STATE_W] << endl;
    for(it=all_inputs.begin() ; it < all_inputs.end(); it++)
    {
        v = fabs(x[STATE_V] + (*it).ctrl[LINEAR_ACCEL]*DELTA_T);
        steering_angle = fabs(x[STATE_STEERING_ANGLE] + (*it).ctrl[ANGULAR_ACCEL]*DELTA_T);
//         cout << "V: " << v << " W: " << w << endl;
        if( v <= max_v && steering_angle <= max_steering_angle )
        {
            inputs.push_back(*it);
        }
    }
//     cout << "Valid Inputs: " << inputs.size() << endl << endl;
}

void CarLikeModel::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[CONTROL_VELOCITY] * cos(x[STATE_THETA]);
    dx[STATE_Y] = u[CONTROL_VELOCITY] * sin(x[STATE_THETA]);
    dx[STATE_THETA] = u[CONTROL_VELOCITY] * m_one_over_bodyLength * tan(u[CONTROL_STEERING_ANGLE]);
}

void CarLikeModel::EstimateNewState(const double *x, const double *u, double *dx)
{
    double w1[3], diffs[2], new_diffs[2], new_v, new_phi;
    int i;
    // Estimando novas velocidades
    memcpy(diffs, x+3, sizeof(double)*2);
    EstimateSpeeds(diffs, u, new_diffs);
    new_v = limit_speed(new_diffs[0], max_v);
    new_phi = limit_steering(new_diffs[1], max_steering_angle);
    new_diffs[0] = new_v;
    new_diffs[1] = new_phi;
    // Atualizando o vetor espaço de estados - (v, angulo de esterçamento)
    memcpy(dx+3, new_diffs, sizeof(double)*2);
    dflow(x, new_diffs, w1);
    // Newton-Euler - Atualizando o vetor espaço de estados - (x,y,theta)
    for(i=0; i<3; i++)
        dx[i] = x[i] + w1[i]*DELTA_T;
}

void CarLikeModel::SpeedFlow(const double *v, const double *u, double *dv)
{
    dv[LINEAR_ACCEL] = u[LINEAR_ACCEL];
    dv[ANGULAR_ACCEL] = u[ANGULAR_ACCEL];
}

void CarLikeModel::EstimateSpeeds(const double *v, const double *u, double *nv)
{
    double w1[2];
    int i;
    // Newton-Euler
    SpeedFlow(v, u, w1);
    for(i=0; i<2; i++)
        nv[i] = v[i] + w1[i]*DELTA_T;
}
