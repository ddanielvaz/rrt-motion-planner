#include "SkidSteerControlBased.hh"
#include "FierroControl.hh"
#include "PDControl.hh"

SkidSteerControlBased::SkidSteerControlBased(double *robot_params, double *speeds_limit, int n_st)
{
    cout << "Criando instancia da classe SkidSteerControlBased." << endl;
    Inertia = robot_params[0];
    mass = robot_params[1];
    fr = robot_params[2];
    mu = robot_params[3];
    xcir = robot_params[4];
    a = robot_params[5];
    b = robot_params[6];
    c = robot_params[7];
    wheel_radius = robot_params[8];
    torque_max = robot_params[9];
    max_v = speeds_limit[0];
    max_w = speeds_limit[1];
    n_states = n_st;
//     trajectory_control = new FierroControl(this);
    trajectory_control = new PDControl(this);
}

void SkidSteerControlBased::GenerateInputs(char *filename)
{
    ifstream accel_inputs(filename);
    char buf[1024], *pt, *acc, *nxt, *dv, *dw;
    control_input temp;
    int i, n_inputs;
    all_inputs.clear();
//     cout << ACCEL_LOGFILE << " has " << n_inputs << " control inputs per line." << endl;
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

void SkidSteerControlBased::GetValidInputs(const double *x)
{
//     cout << "GET VALID INPUTS" << endl << endl ;
    vector<control_input>::iterator it;
    inputs.clear();
    for(it=all_inputs.begin() ; it < all_inputs.end(); it++)
    {
        // Validar limites de velocidad8.0, 10.0, 10.0, 0.01es
        if(VerifyFeasibility(x, (*it).ctrl))
            inputs.push_back(*it);
    }
//     cout << "Valid Inputs: " << inputs.size() << endl << endl;
}

void SkidSteerControlBased::EstimateNewState(const double *x,
                                             const double *ctl, double *dx)
{
    double curr_vel[2], new_vel[2], curr_pos[3], new_pos[3];
    double ideal_states[n_states], inputs[2], computed_torques[2];
    double new_vel_tracked[2], new_pos_tracked[3], tracked_states[n_states];
    // Preenchendo variáveis auxiliares para obtenção do novo estado através de
    // entrada de controle e integração numérica.
    // Copiando as três primeiras posições do vetor do espaço de estados
    // referentes a: (x,y,psi)
    memcpy(curr_pos, x, sizeof(double) * 3);
    // Copiando as duas últimas posições do vetor do espaço de estados
    // referentes a: (v,w)
    memcpy(curr_vel, x+3, sizeof(double) * 2);
    // Estima velocidade futura a partir da velocidade atual e das acelerações
    // linear e angular fornecidas.
    EstimateVelocities(curr_vel, ctl, new_vel);
    // Copiando velocidade estimada para vetor de estados ideais
    memcpy(ideal_states+3, new_vel, sizeof(double) * 2);
    //Estima posição futura a partir da posição atual e da velocidade futura
    // estimada anteriormente.
    EstimatePosition(curr_pos, new_vel, new_pos);
    // Copiando posição estimada para vetor de estados ideais
    memcpy(ideal_states, new_pos, sizeof(double) * 3);
    //XXX: talvez criar um método SimulateRobot para realizar as etapas acima.
    trajectory_control->run(x, ideal_states, inputs);
    
    EstimateTorque(x, inputs, computed_torques);
    EstimateVelocitiesFromTorque(x, computed_torques, new_vel_tracked);
    // Copiando velocidade estimada para vetor de estados acompanhados
    memcpy(tracked_states+3, new_vel_tracked, sizeof(double) * 2);
    //Estima posição futura a partir da posição atual e da velocidade de
    // acompanhamento estimada anteriormente.
    EstimatePosition(curr_pos, new_vel_tracked, new_pos_tracked);
    memcpy(tracked_states, new_pos_tracked, sizeof(double) * 3);
//     cout << "Tracked Vx: " << new_vel_tracked[0] << " Tracked W: " << new_vel_tracked[1] << endl;
//     cout << "Torque_l: " << computed_torques[TORQUE_L] << " Torque_r: " << computed_torques[TORQUE_R] << endl;
//     cout << "#### ENDS ####" << endl << endl;
//     cout << "x: " << new_pos_tracked[0] << " y: " << new_pos_tracked[1] << " psi: " << new_pos_tracked[2] << endl << endl;
    memcpy(dx, tracked_states, sizeof(double)*n_states);
}

void SkidSteerControlBased::dflow(const double *x, const double *ctl, double *dx)
{
    dx[STATE_X] = ctl[VX_SPEED]*cos(x[STATE_THETA]) - xcir * sin(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_Y] = ctl[VX_SPEED]*sin(x[STATE_THETA]) + xcir * cos(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_THETA] = ctl[ANGULAR_SPEED];
}

void SkidSteerControlBased::EstimatePosition(const double *x, const double *speed_ctl, double *dx)
{
    double k1[3], k2[3], k3[3], k4[3], ktemp[3];
    int i;
    dflow(x, speed_ctl, k1);
    for(i=0;i<3;i++)
        ktemp[i] = x[i] + 0.5 * DELTA_T * k1[i];

    dflow(ktemp, speed_ctl, k2);
    for(i=0;i<3;i++)
        ktemp[i] = x[i] + 0.5 * DELTA_T * k2[i];

    dflow(ktemp, speed_ctl, k3);
    for(i=0;i<3;i++)
        ktemp[i] = x[i] + DELTA_T * k3[i];

    dflow(ktemp, speed_ctl, k4);
    for(i=0; i<3; i++)
        dx[i] = x[i] + (DELTA_T/6.0)*(k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

void SkidSteerControlBased::EstimateTorque(const double *x, const double *ctl, double *torques)
{
    double Rx, Fy, Mr, vl, vr, vf, vb, torque_left, torque_right;
    vl = x[STATE_V] + c * x[STATE_W];
    vr = x[STATE_V] - c * x[STATE_W];
    vf = (-xcir + a) * x[STATE_W];
    vb = (-xcir - b) * x[STATE_W];
    Rx = (fr*mass*GRAVITY/2.0)*(sgn(vl)+sgn(vr));
    Fy = mu*((mass*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*mass*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+(fr*c*mass*GRAVITY/2.0)*(sgn(vl)-sgn(vr));
    torque_left = wheel_radius * (Rx + mass * ctl[u1] - x[STATE_W] * mass * x[STATE_W] * xcir)/2.0 +
                  wheel_radius * (Mr + Inertia * ctl[u2] +  Fy * xcir + mass * ctl[u2] * xcir * xcir + x[STATE_W] * mass * x[STATE_V] * xcir)/(2.0*c);

    torque_right = wheel_radius * (Rx + mass * ctl[u1] - x[STATE_W] * mass * x[STATE_W] * xcir)/2.0 -
                  wheel_radius * (Mr + Inertia * ctl[u2] +  Fy * xcir + mass * ctl[u2] * xcir * xcir + x[STATE_W] * mass * x[STATE_V] * xcir)/(2.0*c);
    torques[TORQUE_L] = torque_left;
    torques[TORQUE_R] = torque_right;
}
    
/**
\brief: Este método verifica a factibilidade da entrada de controle, isto é, verifica se os valores de aceleração armazenados em ctl geram torque e velocidade (estimados) dentro dos limites do modelo do robô.
*/
bool SkidSteerControlBased::VerifyFeasibility(const double *x, const double *ctl)
{
    double temp_state[n_states], i_time;
    double estimated_torques[2], curr_vel[2], new_vel[2], new_pos[3];
    bool torque_status, speed_status, status;
    // Faz uma cópia local do conteúdo de x
    memcpy(temp_state, x, sizeof(double) * n_states);
    for(i_time=0.0; i_time<INTEGRATION_TIME; i_time+=DELTA_T)
    {
        EstimateTorque(temp_state, ctl, estimated_torques);
        if(fabs(estimated_torques[TORQUE_L]) <= torque_max && fabs(estimated_torques[TORQUE_R]) <= torque_max)
            torque_status = true;
        else
            torque_status = false;
        // Copiando as duas últimas posições do vetor do espaço de estados
        // referentes a: (v,w)
        memcpy(curr_vel, temp_state+3, sizeof(double) * 2);
        EstimateVelocities(curr_vel, ctl, new_vel);
        if(fabs(new_vel[0]) <= max_v && fabs(new_vel[1]) <= max_w)
            speed_status = true;
        else
            speed_status = false;
        status = torque_status && speed_status;
        if(!status)
            break;
        EstimatePosition(x, new_vel, new_pos);
        // Atualiza o vetor do espaço de estados para próxima iteração.
        memcpy(temp_state, new_pos, sizeof(double) * 3);
        memcpy(temp_state+3, new_vel, sizeof(double) * 2);
    }
    return status;
}

void SkidSteerControlBased::velocities_dflow(const double *x,
                                             const double *ctl,
                                             double *dx)
{
    dx[VX_SPEED] = ctl[LINEAR_ACCEL];
    dx[ANGULAR_SPEED] = ctl[ANGULAR_ACCEL];
}

void SkidSteerControlBased::EstimateVelocities(const double *x,
                                               const double *u_accel,
                                               double *speeds)
{
    double w1[2], initial_state[2];
    int i;
    bzero(speeds, sizeof(double)*2);
    memcpy(initial_state, x, sizeof(double) * 2);
    //Newton-Euler
    velocities_dflow(initial_state, u_accel, w1);
    for(i=0; i<2; i++)
       speeds[i] = initial_state[i] + w1[i] * DELTA_T;
}

void SkidSteerControlBased::EstimateVelocitiesFromTorque(const double *x,
                                                         const double *torque,
                                                         double *speeds)
{
    double Rx, Fy, Mr, vl, vr, vf, vb, dv[2];
    vl = x[STATE_V] + c * x[STATE_W];
    vr = x[STATE_V] - c * x[STATE_W];
    vf = (-xcir + a) * x[STATE_W];
    vb = (-xcir - b) * x[STATE_W];
    Rx = (fr*mass*GRAVITY/2.0)*(sgn(vl)+sgn(vr));
    Fy = mu*((mass*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*mass*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+(fr*c*mass*GRAVITY/2.0)*(sgn(vl)-sgn(vr));

    dv[VX_SPEED] = xcir * x[STATE_W] * x[STATE_W] - Rx/mass + torque[TORQUE_L]/(mass*wheel_radius) + torque[TORQUE_R]/(mass*wheel_radius);
    
    dv[ANGULAR_SPEED] = -(Mr + Fy * xcir + x[STATE_W] * mass * x[STATE_V] * xcir)/(Inertia + mass * xcir * xcir)
                        +(c * torque[TORQUE_L])/((Inertia + mass * xcir * xcir) * wheel_radius)
                        -(c * torque[TORQUE_R])/((Inertia + mass * xcir * xcir) * wheel_radius);

    speeds[VX_SPEED] = x[STATE_V] + dv[VX_SPEED] * DELTA_T;
    speeds[ANGULAR_SPEED] = x[STATE_W] + dv[ANGULAR_SPEED] * DELTA_T;
}

double SkidSteerControlBased::GetXCIR(void)
{
    return this->xcir;
}

SkidSteerControlBased::~SkidSteerControlBased()
{
    cout << "Destruindo instancia da classe SkidSteerControlBased." << endl;
}
