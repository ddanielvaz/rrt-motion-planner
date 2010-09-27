#ifndef _ROBOTS_MODEL_H_
#define _ROBOTS_MODEL_H_

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "constants.h"
#include "math_functions.h"
#include "control.h"

using namespace std;

class RobotModel
{
    public:
        virtual void dflow(const double *x, const double *u, double *dx);
        virtual void EstimateNewState(const double *x,
                                      const double *u, double *dx);
        virtual void GenerateInputs(void);
        virtual void GetValidInputs(const double *x);
        int n_states;
        vector<control_input> inputs;
};

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

class CarLikeModel : public RobotModel
{
    public:
        CarLikeModel(double, double*, int);
        ~CarLikeModel();
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double *x, const double *u, double *dx);
        void SpeedFlow(const double *v, const double *u, double *dv);
        void EstimateSpeeds(const double *v, const double *u, double *nv);
        void GenerateInputs(char *);
        void GetValidInputs(const double *x);
    private:
        double m_one_over_bodyLength, max_v, max_steering_angle;
        vector<control_input> all_inputs;
};

CarLikeModel::CarLikeModel(double body_length, double *constraints, int n_st)
{
    cout << "Criando instancia da classe CarLikeModel" << endl;
    m_one_over_bodyLength = 1.0/body_length;
    max_v = constraints[0];
    max_steering_angle = constraints[1];
    n_states = n_st;
//     double u[42][2]={{0.50, 0.00}, {0.50, 0.02}, {0.50, 0.07}, {0.50, 0.12},
//                      {0.50, 0.17}, {0.50, 0.23}, {0.50, 0.28}, {0.50, 0.33},
//                      {0.50, 0.38}, {0.50, 0.44}, {0.50, 0.49}, {-0.50, 0.00},
//                      {-0.50, 0.02}, {-0.50, 0.07}, {-0.50, 0.12}, {-0.50, 0.17},
//                      {-0.50, 0.23}, {-0.50, 0.28}, {-0.50, 0.33}, {-0.50, 0.38},
//                      {-0.50, 0.44}, {-0.50, 0.49}, {0.50, -0.02}, {0.50, -0.07},
//                      {0.50, -0.12}, {0.50, -0.17}, {0.50, -0.23}, {0.50, -0.28},
//                      {0.50, -0.33}, {0.50, -0.38}, {0.50, -0.44}, {0.50, -0.49},
//                      {-0.50, -0.02}, {-0.50, -0.07}, {-0.50, -0.12},
//                      {-0.50, -0.17}, {-0.50, -0.23}, {-0.50, -0.28},
//                      {-0.50, -0.33}, {-0.50, -0.38}, {-0.50, -0.44},
//                      {-0.50, -0.49} };
//     control_input temp[42];
//     memcpy(temp, u, sizeof(double) * 2 * 42);
//     for (int i=0; i<42; i++)
//         inputs.push_back(temp[i]);
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

class SkidSteerModel : public RobotModel
{
    public:
        SkidSteerModel(int, double);
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double *x,
                              const double *u, double *dx);
        ~SkidSteerModel();
    private:
        double xcir;
};

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

class SkidSteerDynamicModel : public RobotModel
{
    public:
        SkidSteerDynamicModel(double *, double *, double *, int);
        void dflow(const double *, const double *, double *);
        void EstimateNewState(const double *,
                              const double *, double *);
        void velocities_dflow(const double *, const double *, double *);
        void EstimateVelocities(const double *,
                                const double *, double *);
        void GenerateInputs(char *);
        void GetValidInputs(const double *x);
        ~SkidSteerDynamicModel();
    private:
        double Kt, Ke, n, R, I, mass, fr, mu, xcir, a, b, c, wheel_radius,
        max_v, max_w;
        int n_inputs;
        vector<control_input> all_inputs;
};

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

class SkidSteerControlBased : public RobotModel
{
    public:
        SkidSteerControlBased(double *, double *, int);
        void EstimateNewState(const double *, const double *, double *);
        void dflow(const double *, const double *, double *);
        void EstimatePosition(const double *, const double *, double *);
        void velocities_dflow(const double *, const double *, double *);
        void EstimateVelocities(const double *,
                                const double *, double *);
        void GenerateInputs(char *);
        void GetValidInputs(const double *x);
        void EstimateTorque(const double *, const double *, double *);
        bool VerifyFeasibility(const double *x, const double *u);
        ~SkidSteerControlBased();
    private:
        double Inertia, mass, fr, mu, xcir, a, b, c, wheel_radius, torque_max,
               max_v, max_w;
        TrackingControlPioneer3AT trajectory_control;
        vector<control_input> all_inputs;
};

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
        // Validar torques e limites de velocidades
        if(VerifyFeasibility(x, (*it).ctrl))
            inputs.push_back(*it);
    }
//     cout << "Valid Inputs: " << inputs.size() << endl << endl;
}

void SkidSteerControlBased::EstimateNewState(const double *x,
                                             const double *ctl, double *dx)
{
    double curr_vel[2], new_vel[2], curr_pos[3], new_pos[3];
    double ideal_states[n_states], tracked_states[n_states];
    // Preenchendo variáveis auxiliares para obtenção do novo estado através de
    // entrada de controle e integração numérica.
    memcpy(curr_pos, x, sizeof(double) * 3);
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
    trajectory_control.run(x, ideal_states, ctl, tracked_states);
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



void SkidSteerControlBased::velocities_dflow(const double *x,
                                             const double *ctl,
                                             double *dx)
{
    double Rx, Fy, Mr, vl, vr, vf, vb, dtheta, torque_left, torque_right;
    dtheta = x[ANGULAR_SPEED];
    vl = x[VX_SPEED] + c * x[ANGULAR_SPEED];
    vr = x[VX_SPEED] - c * x[ANGULAR_SPEED];
    vf = (-xcir + a) * x[ANGULAR_SPEED];
    vb = (-xcir - b) * x[ANGULAR_SPEED];
    Rx = (fr*mass*GRAVITY/2.0)*(sgn(vl)+sgn(vr));
    Fy = mu*((mass*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*mass*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+(fr*c*mass*GRAVITY/2.0)*(sgn(vl)-sgn(vr));

    torque_left = wheel_radius * (Rx + ctl[DV_DESIRED] * mass - dtheta * mass * x[ANGULAR_SPEED] * xcir)/2 +
                  wheel_radius * (Mr + Inertia * ctl[DW_DESIRED] + xcir * Fy + ctl[DW_DESIRED] *
                  mass * xcir * xcir + dtheta * mass * x[VX_SPEED] * xcir)/(2*c);
    torque_right = wheel_radius * (Rx + ctl[DV_DESIRED] * mass - dtheta * mass * x[ANGULAR_SPEED] * xcir)/2 -
                   wheel_radius * (Mr + Inertia * ctl[DW_DESIRED] + xcir * Fy + ctl[DW_DESIRED] *
                   mass * xcir * xcir + dtheta * mass * x[VX_SPEED] * xcir)/(2*c);

    dx[VX_SPEED] = xcir * dtheta * x[ANGULAR_SPEED] - Rx/mass + torque_left/(mass*wheel_radius) + torque_right/(mass*wheel_radius);
    dx[ANGULAR_SPEED] = -(mass*xcir*x[ANGULAR_SPEED]*x[VX_SPEED])/(mass*xcir*xcir+Inertia)
                        -(Mr+xcir*Fy)/(mass*xcir*xcir+Inertia)
                        +(c*torque_left)/((mass*xcir*xcir+Inertia)*wheel_radius)
                        -(c*torque_right)/((mass*xcir*xcir+Inertia)*wheel_radius);
}

void SkidSteerControlBased::EstimateTorque(const double *x, const double *ctl, double *torques)
{
    double Rx, Fy, Mr, vl, vr, vf, vb, dtheta, torque_left, torque_right;
    dtheta = x[ANGULAR_SPEED];
    vl = x[VX_SPEED] + c * x[ANGULAR_SPEED];
    vr = x[VX_SPEED] - c * x[ANGULAR_SPEED];
    vf = (-xcir + a) * x[ANGULAR_SPEED];
    vb = (-xcir - b) * x[ANGULAR_SPEED];
    Rx = (fr*mass*GRAVITY/2.0)*(sgn(vl)+sgn(vr));
    Fy = mu*((mass*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*mass*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+(fr*c*mass*GRAVITY/2.0)*(sgn(vl)-sgn(vr));
    torque_left = wheel_radius * (Rx + ctl[DV_DESIRED] * mass - dtheta * mass * x[ANGULAR_SPEED] * xcir)/2 +
                  wheel_radius * (Mr + Inertia * ctl[DW_DESIRED] + xcir * Fy + ctl[DW_DESIRED] *
                  mass * xcir * xcir + dtheta * mass * x[VX_SPEED] * xcir)/(2*c);
    torque_right = wheel_radius * (Rx + ctl[DV_DESIRED] * mass - dtheta * mass * x[ANGULAR_SPEED] * xcir)/2 -
                   wheel_radius * (Mr + Inertia * ctl[DW_DESIRED] + xcir * Fy + ctl[DW_DESIRED] *
                   mass * xcir * xcir + dtheta * mass * x[VX_SPEED] * xcir)/(2*c);
    torques[TORQUE_L] = torque_left;
    torques[TORQUE_R] = torque_right;
}
    
/**
@brief: Este método verifica a factibilidade da entrada de controle, isto é, verifica se os valores de aceleração armazenados em ctl geram torque e velocidade (estimados) dentro dos limites do modelo do robô.
*/
bool SkidSteerControlBased::VerifyFeasibility(const double *x, const double *ctl)
{
    
    double estimated_torques[2], curr_vel[2], new_vel[2];
    bool torque_status=false, speed_status=false, status;
    EstimateTorque(x, ctl, estimated_torques);
//     cout << "T_left: " << estimated_torques[TORQUE_L] << " T_right: " << estimated_torques[TORQUE_R] << endl;
    if( fabs(estimated_torques[TORQUE_L]) <= torque_max && fabs(estimated_torques[TORQUE_R]) <= torque_max)
        torque_status = true;
    memcpy(curr_vel, x+3, sizeof(double) * 2);
    EstimateVelocities(curr_vel, ctl, new_vel);
//     cout << "VX: " << new_vel[0] << " W: " << new_vel[1] << endl;
    if(fabs(new_vel[0]) <= max_v && fabs(new_vel[1]) <= max_w)
        speed_status = true;
    status = torque_status && speed_status;
//     cout << "The status is: " << status << endl;
    return status;
}

void SkidSteerControlBased::EstimateVelocities(const double *x,
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

SkidSteerControlBased::~SkidSteerControlBased()
{
    cout << "Destruindo instancia da classe SkidSteerControlBased." << endl;
}

class PioneerCarLikeModel: public RobotModel
{
    public:
        PioneerCarLikeModel(double *, double *, double *, int);
        void dflow(const double *, const double *, double *);
        void EstimateNewState(const double *,
                              const double *, double *);
        void velocities_dflow(const double *, const double *, double *);
        void EstimateVelocities(const double *,
                                const double *, double *);
        ~PioneerCarLikeModel();
    private:
        double Kt, Ke, n, R, I, m, fr, mu, xcir, a, b, c, r, max_v, max_w, m_one_over_bodyLength;
};

PioneerCarLikeModel::PioneerCarLikeModel(double *motor_params, double *robot_params, double *speeds_limit, int n_st)
{
    cout << "Criando instancia da classe PioneerCarLike." << endl;
    Kt = motor_params[0];
    Ke = motor_params[1];
    n = motor_params[2];
    R = motor_params[3];
    I = robot_params[0];
    m = robot_params[1];
    fr = robot_params[2];
    mu = robot_params[3];
    xcir = robot_params[4];
    a = robot_params[5];
    b = robot_params[6];
    c = robot_params[7];
    r = robot_params[8];
    max_v = speeds_limit[0];
    max_w = speeds_limit[1];
    n_states = n_st;
    m_one_over_bodyLength = a+b;
    #define entradas 12
    double u[entradas][2]={{8, 8}, {6, 6}, {6, 8}, {8, 6}, {-6,-6}, {-6, -8},
                           {-8, -6}, {-2,-2},{-4,-4},{-5,-5},{0,-2},{-2,0}};
    control_input temp[entradas];
    memcpy(temp, u, sizeof(double) * entradas * 2);
    for(int i=0; i<entradas; i++)
        inputs.push_back(temp[i]);
}

void PioneerCarLikeModel::dflow(const double *x, const double *ctl, double *dx)
{
    dx[STATE_X] = ctl[VX_SPEED]*cos(x[STATE_THETA]) - xcir * sin(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_Y] = ctl[VX_SPEED]*sin(x[STATE_THETA]) + xcir * cos(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_THETA] = ctl[ANGULAR_SPEED];
}

void PioneerCarLikeModel::EstimateNewState(const double *x,
                                             const double *ctl, double *dx)
{
    double curr_vel[2], new_vel[2], w1[3], w2[3], w3[3], w4[3], wtemp[3];
    int i;
    memcpy(curr_vel, x+3, sizeof(double) * 2);
    EstimateVelocities(curr_vel, ctl, new_vel);
    new_vel[0] = limit_speed(new_vel[0], max_v);
    new_vel[1] = limit_speed(new_vel[1], max_w);
    memcpy(dx+3, new_vel, sizeof(double)*2);

    dflow(x, new_vel, w1);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * DELTA_T * w1[i];

    dflow(wtemp, new_vel, w2);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * DELTA_T * w2[i];

    dflow(wtemp, new_vel, w3);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + DELTA_T * w3[i];

    dflow(wtemp, new_vel, w4);
    for(i=0; i<3; i++)
        dx[i] = x[i] + (DELTA_T/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
    
    //Newton-Euler
/*    for(i=0; i<3; i++)
       dx[i] = x[i] + w1[i]*t;*/
}

void PioneerCarLikeModel::velocities_dflow(const double *x,
                                             const double *ctl,
                                             double *dx)
{
    double Rx, Fy, Mr, vl, vr, vf, vb, dtheta;
    dtheta = x[ANGULAR_SPEED];
    vl = x[VX_SPEED] + c * x[ANGULAR_SPEED];
    vr = x[VX_SPEED] - c * x[ANGULAR_SPEED];
    vf = (-xcir + a) * x[ANGULAR_SPEED];
    vb = (-xcir - b) * x[ANGULAR_SPEED];
    Fy = mu*((m*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*m*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+fr*c*m*GRAVITY*(sgn(vl)-sgn(vr));
    Rx = fr*m*GRAVITY*(sgn(vl)+sgn(vr));
    //cout << Rx << endl;
    dx[VX_SPEED] = xcir * dtheta * x[ANGULAR_SPEED] - Rx/m + ctl[TORQUE_L]/(m*r) + ctl[TORQUE_R]/(m*r);
    dx[ANGULAR_SPEED] = -(m*xcir*x[ANGULAR_SPEED]*x[VX_SPEED])/(m*xcir*xcir+I)
                        -(Mr+xcir*Fy)/(m*xcir*xcir+I)
                        +(c*ctl[TORQUE_L])/((m*xcir*xcir+I)*r)
                        -(c*ctl[TORQUE_R])/((m*xcir*xcir+I)*r);
}

void PioneerCarLikeModel::EstimateVelocities(const double *x,
                                               const double *u_torque,
                                               double *speeds)
{
    double w1[2], w2[2], w3[2], w4[2], wtemp[2], initial_state[2];
    int i;
    bzero(speeds, sizeof(double)*2);
    memcpy(initial_state, x, sizeof(double) * 2);
    //Runge Kutta 4th
    velocities_dflow(initial_state, u_torque, w1);
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + 0.5 * DELTA_T * w1[i];

    velocities_dflow(wtemp, u_torque, w2);
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + 0.5 * DELTA_T * w2[i];

    velocities_dflow(wtemp, u_torque, w3);
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + DELTA_T * w3[i];

    velocities_dflow(wtemp, u_torque, w4);
    for(i=0; i<2; i++)
        speeds[i] = initial_state[i] + (DELTA_T/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
    //Newton-Euler
/*    velocities_dflow(initial_state, u_torque, w1);
    for(i=0; i<2; i++)
       speeds[i] = initial_state[i] + w1[i]*t;*/
}

PioneerCarLikeModel::~PioneerCarLikeModel()
{
    cout << "Destruindo instancia da classe PioneerCarLike." << endl;
}

#endif
