#ifndef _ROBOTS_MODEL_H_
#define _ROBOTS_MODEL_H_

#include <math.h>
#include <string.h>
#include <iostream>

#include "utils.h"

using namespace std;

typedef struct
{
    double ctrl[2];
}control_input;

class RobotModel
{
    public:
        virtual void dflow(const double *x, const double *u, double *dx);
        virtual void EstimateNewState(const double dt, const double *x,
                                      const double *u, double *dx);
        int n_states;
        vector<control_input> inputs;
};

void RobotModel::dflow(const double *x, const double *u, double *dx)
{
    cout << "VIRTUAL METHOD dflow." << endl;
}

void RobotModel::EstimateNewState(const double dt, const double *x,
                                          const double *u, double *dx)
{
    cout << "VIRTUAL METHOD EstimateNewState." << endl;
}

class CarLikeModel : public RobotModel
{
    public:
        CarLikeModel(double, int);
        ~CarLikeModel();
        /**
          *@author Erion Plaku
          *@brief Definition of the differential flow as a set of ODEs
          *@param x state values at time <em>t</em>
          *@param t current time
          *@param params external parameters needed to compute
                    derivative (control values)
          *@param dx derivative of state at time <em>t</em>
        */
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double dt, const double *x,
                              const double *u, double *dx);
    private:
        double m_one_over_bodyLength;
};

CarLikeModel::CarLikeModel(double bodyLength, int n_st)
{
    cout << "Criando instancia da classe CarLikeModel" << endl;
    m_one_over_bodyLength = 1.0/bodyLength;
    n_states = n_st;
    double u[42][2]={{0.50, 0.00}, {0.50, 0.02}, {0.50, 0.07}, {0.50, 0.12},
                     {0.50, 0.17}, {0.50, 0.23}, {0.50, 0.28}, {0.50, 0.33},
                     {0.50, 0.38}, {0.50, 0.44}, {0.50, 0.49}, {-0.50, 0.00},
                     {-0.50, 0.02}, {-0.50, 0.07}, {-0.50, 0.12}, {-0.50, 0.17},
                     {-0.50, 0.23}, {-0.50, 0.28}, {-0.50, 0.33}, {-0.50, 0.38},
                     {-0.50, 0.44}, {-0.50, 0.49}, {0.50, -0.02}, {0.50, -0.07},
                     {0.50, -0.12}, {0.50, -0.17}, {0.50, -0.23}, {0.50, -0.28},
                     {0.50, -0.33}, {0.50, -0.38}, {0.50, -0.44}, {0.50, -0.49},
                     {-0.50, -0.02}, {-0.50, -0.07}, {-0.50, -0.12},
                     {-0.50, -0.17}, {-0.50, -0.23}, {-0.50, -0.28},
                     {-0.50, -0.33}, {-0.50, -0.38}, {-0.50, -0.44},
                     {-0.50, -0.49} };
    control_input temp[42];
    memcpy(temp, u, sizeof(double) * 2 * 42);
    for (int i=0; i<42; i++)
        inputs.push_back(temp[i]);
}

CarLikeModel::~CarLikeModel()
{
    cout << "Destruindo instancia da classe CarLikeModel" << endl;
}

void CarLikeModel::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[CONTROL_VELOCITY] * cos(x[STATE_THETA]);
    dx[STATE_Y] = -u[CONTROL_VELOCITY] * sin(x[STATE_THETA]);
    dx[STATE_THETA] = u[CONTROL_STEERING_ANGLE];//u[CONTROL_VELOCITY] * m_one_over_bodyLength * tan(u[CONTROL_STEERING_ANGLE]);
}

void CarLikeModel::EstimateNewState(const double dt, const double *x,
                                const double *u, double *dx)
{
    double w1[3], w2[3], w3[3], w4[3], wtemp[3];
    int i;
    //cout << "vel: "<<u[CONTROL_VELOCITY]<<" steer: "<< u[CONTROL_STEERING_ANGLE] << endl;
    
    dflow(x, u, w1);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * dt * w1[i];
    dflow(wtemp, u, w2);

    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * dt * w2[i];
    dflow(wtemp, u, w3);

    for(i=0;i<3;i++)
        wtemp[i] = x[i] + dt * w3[i];
    dflow(wtemp, u, w4);

    for(i=0; i<3; i++)
        dx[i] = x[i] + (dt/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
}

class SkidSteerModel : public RobotModel
{
    public:
        SkidSteerModel(int);
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double dt, const double *x,
                              const double *u, double *dx);
        ~SkidSteerModel();
    private:
        static const double xcir = 0.008;
};

SkidSteerModel::SkidSteerModel(int n_st)
{
    cout << "Criando instancia da classe SkidSteerModel." << endl;
    n_states = n_st;
    double u[42][2]={{0.50, 0.00}, {0.50, 0.02}, {0.50, 0.07}, {0.50, 0.12},
                     {0.50, 0.17}, {0.50, 0.23}, {0.50, 0.28}, {0.50, 0.33},
                     {0.50, 0.38}, {0.50, 0.44}, {0.50, 0.49}, {-0.50, 0.00},
                     {-0.50, 0.02}, {-0.50, 0.07}, {-0.50, 0.12}, {-0.50, 0.17},
                     {-0.50, 0.23}, {-0.50, 0.28}, {-0.50, 0.33}, {-0.50, 0.38},
                     {-0.50, 0.44}, {-0.50, 0.49}, {0.50, -0.02}, {0.50, -0.07},
                     {0.50, -0.12}, {0.50, -0.17}, {0.50, -0.23}, {0.50, -0.28},
                     {0.50, -0.33}, {0.50, -0.38}, {0.50, -0.44}, {0.50, -0.49},
                     {-0.50, -0.02}, {-0.50, -0.07}, {-0.50, -0.12},
                     {-0.50, -0.17}, {-0.50, -0.23}, {-0.50, -0.28},
                     {-0.50, -0.33}, {-0.50, -0.38}, {-0.50, -0.44},
                     {-0.50, -0.49} };
    control_input temp[42];
    memcpy(temp, u, sizeof(double) * 2 * 42);
    for (int i=0; i<42; i++)
        inputs.push_back(temp[i]);
}

void SkidSteerModel::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[VX_SPEED] * cos(x[STATE_THETA]) - xcir * sin(x[STATE_THETA]) * u[ANGULAR_SPEED];
    dx[STATE_Y] = -u[VX_SPEED] * sin(x[STATE_THETA]) - xcir * cos(x[STATE_THETA]) * u[ANGULAR_SPEED];
    dx[STATE_THETA] = u[ANGULAR_SPEED];
}

void SkidSteerModel::EstimateNewState(const double dt, const double *x,
                                      const double *u, double *dx)
{
    double w1[3], w2[3], w3[3], w4[3], wtemp[3];
    int i;

    dflow(x, u, w1);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * dt * w1[i];
    dflow(wtemp, u, w2);

    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * dt * w2[i];
    dflow(wtemp, u, w3);

    for(i=0;i<3;i++)
        wtemp[i] = x[i] + dt * w3[i];
    dflow(wtemp, u, w4);

    for(i=0; i<3; i++)
        dx[i] = x[i] + (dt/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
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
        void EstimateNewState(const double , const double *,
                              const double *, double *);
        void velocities_dflow(const double *, const double *, double *);
        void EstimateVelocities(const double , const double *,
                                const double *, double *);
        ~SkidSteerDynamicModel();
    private:
        double Kt, Ke, n, R, I, m, fr, mu, xcir, a, b, c, r, max_v, max_w;
};

SkidSteerDynamicModel::SkidSteerDynamicModel(double *motor_params, double *robot_params, double *speeds_limit, int n_st)
{
    cout << "Criando instancia da classe SkidSteerDynamicModel." << endl;
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
    double u[12][2]={{10, 10}, {-10, -10}, {10, 5},
    {-10, -5}, {-10, 5}, {10, -5}, {5, 10}, {-5,-10}, {-5, 10}, {5, -10},
    {5, 5}, {-5, -5}};
    control_input temp[12];
    memcpy(temp, u, sizeof(double) * 12 * 2);
    for(int i=0; i<12; i++)
        inputs.push_back(temp[i]);
}

void SkidSteerDynamicModel::dflow(const double *x, const double *ctl, double *dx)
{
    dx[STATE_X] = ctl[VX_SPEED]*cos(x[STATE_THETA]) + xcir * sin(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_Y] = ctl[VX_SPEED]*sin(x[STATE_THETA]) - xcir * cos(x[STATE_THETA]) * ctl[ANGULAR_SPEED];
    dx[STATE_THETA] = ctl[ANGULAR_SPEED];
}

void SkidSteerDynamicModel::EstimateNewState(const double t, const double *x,
                                             const double *ctl, double *dx)
{
    double curr_vel[2], new_vel[2], w1[3], w2[3], w3[3], w4[3], wtemp[3];
    int i;
    memcpy(curr_vel, x+3, sizeof(double) * 2);
    EstimateVelocities(t, curr_vel, ctl, new_vel);
    new_vel[0] = limit_speed(new_vel[0], max_v);
    new_vel[1] = limit_speed(new_vel[1], max_w);
    memcpy(dx+3, new_vel, sizeof(double)*2);

    dflow(x, new_vel, w1);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * t * w1[i];

    dflow(wtemp, new_vel, w2);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + 0.5 * t * w2[i];

    dflow(wtemp, new_vel, w3);
    for(i=0;i<3;i++)
        wtemp[i] = x[i] + t * w3[i];

    dflow(wtemp, new_vel, w4);
    for(i=0; i<3; i++)
        dx[i] = x[i] + (t/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
    
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
    vl = x[VX_SPEED] - c * x[ANGULAR_SPEED];
    vr = x[VX_SPEED] + c * x[ANGULAR_SPEED];
    vf = (-xcir + b) * x[ANGULAR_SPEED];
    vb = (-xcir - a) * x[ANGULAR_SPEED];
    Fy = mu*((m*GRAVITY)/(a+b))*(b*sgn(vf)+a*sgn(vb));
    Mr = mu*((a*b*m*GRAVITY)/(a+b))*(sgn(vf)-sgn(vb))+fr*c*m*GRAVITY*(sgn(vr)-sgn(vl));
    Rx = fr*m*GRAVITY*(sgn(vl)+sgn(vr));
    //cout << Rx << endl;
    dx[VX_SPEED] = -xcir * dtheta * x[ANGULAR_SPEED] - Rx/m + ctl[TORQUE_L]/(m*r) + ctl[TORQUE_R]/(m*r);
    dx[ANGULAR_SPEED] = (m*xcir*dtheta*x[VX_SPEED])/(m*xcir*xcir+I) -
                        (Mr-xcir*Fy)/(m*xcir*xcir+I)
                        - (c*ctl[TORQUE_L])/((m*xcir*xcir+I)*r)
                        + (c*ctl[TORQUE_R])/((m*xcir*xcir+I)*r);
}

void SkidSteerDynamicModel::EstimateVelocities(const double t, const double *x,
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
        wtemp[i] = initial_state[i] + 0.5 * t * w1[i];

    velocities_dflow(wtemp, u_torque, w2);
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + 0.5 * t * w2[i];

    velocities_dflow(wtemp, u_torque, w3);
    for(i=0;i<2;i++)
        wtemp[i] = initial_state[i] + t * w3[i];

    velocities_dflow(wtemp, u_torque, w4);
    for(i=0; i<2; i++)
        speeds[i] = initial_state[i] + (t/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
    //Newton-Euler
/*    velocities_dflow(initial_state, u_torque, w1);
    for(i=0; i<2; i++)
       speeds[i] = initial_state[i] + w1[i]*t;*/
}

SkidSteerDynamicModel::~SkidSteerDynamicModel()
{
    cout << "Destruindo instancia da classe SkidSteerDynamicModel." << endl;
}

#endif
