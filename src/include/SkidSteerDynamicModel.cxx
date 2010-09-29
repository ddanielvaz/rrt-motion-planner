#include "SkidSteerDynamicModel.hh"

SkidSteerDynamicModel::SkidSteerDynamicModel(double *motor_params, double *robot_params, double *speeds_limit, int n_st)
{
    cout << "Criando instancia da classe SkidSteerDynamicModel." << endl;
    Kt = motor_params[0];
    Ke = motor_params[1];
    n = motor_params[2];
    R = motor_params[3];
    I = robot_params[0];
    mass = robot_params[1];
    fr = robot_params[2];
    mu = robot_params[3];
    xcir = robot_params[4];
    a = robot_params[5];
    b = robot_params[6];
    c = robot_params[7];
    wheel_radius = robot_params[8];
    max_v = speeds_limit[0];
    max_w = speeds_limit[1];
    n_states = n_st;
}

void SkidSteerDynamicModel::GenerateInputs(char *filename)
{
    ifstream ctrl_inputs_fp(filename);
    char buf[1024], *pt, *torques, *nxt, *t_l, *t_r;
    control_input temp;
    int i;
    all_inputs.clear();
    ctrl_inputs_fp.getline(buf, 1024);
    n_inputs = atoi(buf);
    cout << filename << " has " << n_inputs << " control inputs per line." << endl;
    while(ctrl_inputs_fp.getline(buf, 1024))
    {
        pt = buf;
        for(i=0;i<n_inputs;i++)
        {
            torques = strtok_r(pt, ";", &nxt);
            t_r = strtok_r(torques, ",", &t_l);
            temp.ctrl[TORQUE_R] = atof(t_r)/2.0;
            temp.ctrl[TORQUE_L] = atof(t_l)/2.0;
            cout << "Torque right: " << temp.ctrl[TORQUE_R] << " Torque left: " << temp.ctrl[TORQUE_L] << endl;
            all_inputs.push_back(temp);
            pt = nxt;
        }
    }
}

void SkidSteerDynamicModel::GetValidInputs(const double *x)
{
    double speed_step = 0.05, quant_speed;
    int initial_index, final_index, i;
    quant_speed = nearest_quantified_speed(x[STATE_V], speed_step);
    initial_index = ceil((quant_speed+0.5)/speed_step) * n_inputs;
    final_index = initial_index + n_inputs;
//     cout << "Current speed: " << x[STATE_V] << " quantic speed: " << quant_speed << endl;
//     cout << "Adding elements from index " << initial_index << " until index " << final_index << endl;
    inputs.clear();
    for(i=initial_index; i<final_index; i++)
        inputs.push_back(all_inputs.at(i));
}

void SkidSteerDynamicModel::dflow(const double *x, const double *ctl, double *dx)
{
    dx[STATE_X] = ctl[VX_SPEED]*cos(x[STATE_THETA]) - xcir * sin(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_Y] = ctl[VX_SPEED]*sin(x[STATE_THETA]) + xcir * cos(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_THETA] = ctl[ANGULAR_SPEED];
}

void SkidSteerDynamicModel::EstimateNewState(const double *x,
                                             const double *ctl, double *dx)
{
    double curr_vel[2], new_vel[2];
    double k1[3], k2[3], k3[3], k4[3], ktemp[3];
    int i;
    memcpy(curr_vel, x+3, sizeof(double) * 2);
    EstimateVelocities(curr_vel, ctl, new_vel);
    // Atualizando vetor do espaço de estados - (v,w)
    memcpy(dx+3, new_vel, sizeof(double)*2);

    dflow(x, new_vel, k1);
    for(i=0;i<3;i++)
        ktemp[i] = x[i] + 0.5 * DELTA_T * k1[i];

    dflow(ktemp, new_vel, k2);
    for(i=0;i<3;i++)
        ktemp[i] = x[i] + 0.5 * DELTA_T * k2[i];

    dflow(ktemp, new_vel, k3);
    for(i=0;i<3;i++)
        ktemp[i] = x[i] + DELTA_T * k3[i];

    dflow(ktemp, new_vel, k4);
    for(i=0; i<3; i++)
        dx[i] = x[i] + (DELTA_T/6.0)*(k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    
    //Newton-Euler
/*    for(i=0; i<3; i++)
       dx[i] = x[i] + w1[i]*t;*/
}

void SkidSteerDynamicModel::velocities_dflow(const double *x,
                                             const double *ctl,
                                             double *dx)
{
    double Rx, Fy, Mr, vl, vr, vf, vb, dtheta;
    dtheta = x[ANGULAR_SPEED];
    vl = x[VX_SPEED] + c * x[ANGULAR_SPEED];
    vr = x[VX_SPEED] - c * x[ANGULAR_SPEED];
    vf = (-xcir + a) * x[ANGULAR_SPEED];
    vb = (-xcir - b) * x[ANGULAR_SPEED];
//     cout << "vl: " << vl;
//     cout << " vr: " << vr;
//     cout << " vf: " << vf;
//     cout << " vb: " << vb;
    Fy = mu*((mass*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*mass*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+(fr*c*mass*GRAVITY/2.0)*(sgn(vl)-sgn(vr));
    Rx = (fr*mass*GRAVITY/2.0)*(sgn(vl)+sgn(vr));
//     cout << "Rx: " << Rx << endl;
    dx[VX_SPEED] = xcir * dtheta * x[ANGULAR_SPEED] - Rx/mass + ctl[TORQUE_L]/(mass*wheel_radius) + ctl[TORQUE_R]/(mass*wheel_radius);
    //Limitar aceleração
    dx[ANGULAR_SPEED] = -(mass*xcir*x[ANGULAR_SPEED]*x[VX_SPEED])/(mass*xcir*xcir+I)
                        -(Mr+xcir*Fy)/(mass*xcir*xcir+I)
                        +(c*ctl[TORQUE_L])/((mass*xcir*xcir+I)*wheel_radius)
                        -(c*ctl[TORQUE_R])/((mass*xcir*xcir+I)*wheel_radius);
//     cout << " dv: " << dx[VX_SPEED] << " dw: " << dx[ANGULAR_SPEED] << endl;
}

void SkidSteerDynamicModel::EstimateVelocities(const double *x,
                                               const double *u_torque,
                                               double *speeds)
{
    double w1[2], w2[2], w3[2], w4[2], wtemp[2], initial_state[2];
    int i;
    bzero(speeds, sizeof(double)*2);
    memcpy(initial_state, x, sizeof(double) * 2);
    //Runge Kutta 4th
    velocities_dflow(initial_state, u_torque, w1);
//     cout << "k1_dv: " << w1[0] << " k1_dw: " << w1[1] << endl;
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + 0.5 * DELTA_T * w1[i];

    velocities_dflow(wtemp, u_torque, w2);
//     cout << "k2_dv: " << w2[0] << " k2_dw: " << w2[1] << endl;
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + 0.5 * DELTA_T * w2[i];

    velocities_dflow(wtemp, u_torque, w3);
//     cout << "k3_dv: " << w3[0] << " k3_w: " << w3[1] << endl;
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + DELTA_T * w3[i];

    velocities_dflow(wtemp, u_torque, w4);
//     cout << "k4_dv: " << w4[0] << " k4_dw: " << w4[1] << endl;
    for(i=0; i<2; i++)
        speeds[i] = initial_state[i] + (DELTA_T/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);

    //Newton-Euler
/*    velocities_dflow(initial_state, u_torque, w1);
    for(i=0; i<2; i++)
       speeds[i] = initial_state[i] + w1[i]*t;*/
}

SkidSteerDynamicModel::~SkidSteerDynamicModel()
{
    cout << "Destruindo instancia da classe SkidSteerDynamicModel." << endl;
}
