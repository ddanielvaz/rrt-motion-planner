#ifndef _ROBOTS_MODEL_H_
#define _ROBOTS_MODEL_H_

#include <math.h>
#include <string.h>
#include <iostream>

#include "utils.h"

using namespace std;

class RobotModel
{
    public:
        virtual void dflow(const double *x, const double *u, double *dx);
        virtual void EstimateNewState(const double dt, const double *x,
                                      const double *u, double *dx);
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
        CarLikeModel(double);
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

CarLikeModel::CarLikeModel(double bodyLength)
{
    cout << "Criando instancia da classe CarLikeModel" << endl;
    m_one_over_bodyLength = 1.0/bodyLength;
}

CarLikeModel::~CarLikeModel()
{
    cout << "Destruindo instancia da classe CarLikeModel" << endl;
}

void CarLikeModel::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[CONTROL_VELOCITY] * cos(x[STATE_THETA]);
    dx[STATE_Y] = u[CONTROL_VELOCITY] * sin(x[STATE_THETA]);
    dx[STATE_THETA] = u[CONTROL_VELOCITY] * m_one_over_bodyLength * tan(u[CONTROL_STEERING_ANGLE]);
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
        SkidSteerModel();
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double dt, const double *x,
                              const double *u, double *dx);
        ~SkidSteerModel();
    private:
        static const double xcir = 0.008;
};

SkidSteerModel::SkidSteerModel()
{
    cout << "Criando instancia da classe SkidSteerModel." << endl;
}

void SkidSteerModel::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[VX_SPEED] * cos(x[STATE_THETA]) + xcir * sin(x[STATE_THETA]) * u[ANGULAR_SPEED];
    dx[STATE_Y] = u[VX_SPEED] * sin(x[STATE_THETA]) - xcir * cos(x[STATE_THETA]) * u[ANGULAR_SPEED];
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
        SkidSteerDynamicModel(double *, double *);
        void dflow(const double *, const double *, double *);
        void EstimateNewState(const double , const double *,
                              const double *, double *);
        void control_velocities_dflow(const double *, const double *, double *);
        void velocities_dflow(const double *, const double *, double *);
        void EstimateControlVelocities(const double , const double *,
                                const double *, double *);
        void EstimateVelocities(const double , const double *,
                                const double *, double *);
        ~SkidSteerDynamicModel();
    private:
        double Kt, Ke, n, R, I, m, fr, mu, xcir, a, b, c, r;
};

SkidSteerDynamicModel::SkidSteerDynamicModel(double *motor_params, double *robot_params)
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
}

void SkidSteerDynamicModel::dflow(const double *x, const double *ctl, double *dx)
{
    dx[STATE_X] = 0.0;
    dx[STATE_Y] = 0.0;
    dx[STATE_THETA] = 0.0;
    /*dx[STATE_V] = -xcir * dx[STATE_THETA] * x[STATE_W];
    dx[STATE_W] = (m*xcir*dx[STATE_THETA]*x[STATE_V])/(m*xcir*xcir+I) -
                  (Mr-xcir*Fy)/(m*xcir*xcir+I) -
                  (c*torque_left)/((m*xcir*xcir+I)*r) +
                  (c*torque_right)/((m*xcir*xcir+I)*r);*/
}

void SkidSteerDynamicModel::EstimateNewState(const double dt, const double *x,
                                             const double *ctl, double *dx)
{
    double w1[3], w2[3], w3[3], w4[3], wtemp[3];
    double ctrl_vel[2], vel[2], u_control[2], curr_vel[2];
    double w_left, w_right, torque_left, torque_right, u=12;
    int i;
    memcpy(curr_vel, x+3, sizeof(double) * 2);
    EstimateControlVelocities(dt, curr_vel, ctl, ctrl_vel);
    cout << "v_ctrl: " << ctrl_vel[0] << endl;
    cout << "w_ctrl: " << ctrl_vel[1] << endl;
    // Calculando w_left e w_right
    w_left = ctrl_vel[VX_SPEED]/r - c*ctrl_vel[ANGULAR_SPEED]/r;
    w_right = ctrl_vel[VX_SPEED]/r + c*ctrl_vel[ANGULAR_SPEED]/r;
    // Calculando torque_left e torque_right
    torque_left = 2 * n * Kt * (u - n * Ke * w_left);
    torque_right = 2 * n * Kt * (u - n * Ke * w_right);
    cout << "torque_left: " << torque_left << endl;
    cout << "torque_right: " << torque_right << endl;
    u_control[TORQUE_R] = torque_right;
    u_control[TORQUE_L] = torque_left;
    EstimateVelocities(dt, curr_vel, u_control, vel);
    cout << "v: " << vel[0] << endl;
    cout << "w: " << vel[1] << endl;
    
    /*dflow(x, u, w1);
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
        dx[i] = x[i] + (dt/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);*/
}

void SkidSteerDynamicModel::velocities_dflow(const double *x,
                                             const double *ctl,
                                             double *dx)
{
    double Rx, Fy, Mr, vx1, vx2, vy1, vy3, w, dtheta, vx, torque_left, torque_right;
    torque_right = ctl[TORQUE_R];
    torque_left = ctl[TORQUE_L];
    vx = x[STATE_V];
    w = dtheta = x[STATE_W];
    vx1 = vx - c * w;
    vx2 = vx + c * w;
    vy1 = (-xcir + b) * w;
    vy3 = (-xcir - a) * w;
    Fy = mu*((m*GRAVITY)/(a+b))*(b*sgn(vy1)+a*sgn(vy3));
    Mr = mu*((a*b*m*GRAVITY)/(a+b))*(sgn(vy1)-sgn(vy3))+fr*c*m*GRAVITY*(sgn(vx2)-sgn(vx1));
    Rx = fr*m*GRAVITY*(sgn(vx1)+sgn(vx2));
    dx[VX_SPEED] = -xcir * dtheta * w - Rx/m + torque_left/(m*r) + torque_right/(m*r);
    dx[ANGULAR_SPEED] = (m*xcir*dtheta*vx)/(m*xcir*xcir+I) -
                        (Mr-xcir*Fy)/(m*xcir*xcir+I)
                        - (c*torque_left)/((m*xcir*xcir+I)*r)
                        + (c*torque_right)/((m*xcir*xcir+I)*r);
}


void SkidSteerDynamicModel::control_velocities_dflow(const double *x,
                                                     const double *u,
                                                     double *dx)
{
    dx[VX_SPEED] = u[LINEAR_ACCEL];
    dx[ANGULAR_SPEED] = u[ANGULAR_ACCEL];
}

void SkidSteerDynamicModel::EstimateControlVelocities(const double dt,
                                                      const double *x,
                                                      const double *u_accel,
                                                      double *speeds)
{
    double w1[2], w2[2], w3[2], w4[2], wtemp[2];
    int i;

    control_velocities_dflow(x, u_accel, w1);
    for(i=0;i<2;i++)
        wtemp[i] = x[i] + 0.5 * dt * w1[i];
    control_velocities_dflow(wtemp, u_accel, w2);

    for(i=0;i<2;i++)
        wtemp[i] = x[i] + 0.5 * dt * w2[i];
    control_velocities_dflow(wtemp, u_accel, w3);

    for(i=0;i<2;i++)
        wtemp[i] = x[i] + dt * w3[i];
    control_velocities_dflow(wtemp, u_accel, w4);

    for(i=0; i<2; i++)
        speeds[i] = x[i] + (dt/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
}

void SkidSteerDynamicModel::EstimateVelocities(const double dt, const double *x,
                                               const double *u_torque,
                                               double *speeds)
{
    double w1[2], w2[2], w3[2], w4[2], wtemp[2];
    int i;

    velocities_dflow(x, u_torque, w1);
    for(i=0;i<2;i++)
        wtemp[i] = x[i] + 0.5 * dt * w1[i];
    velocities_dflow(wtemp, u_torque, w2);

    for(i=0;i<2;i++)
        wtemp[i] = x[i] + 0.5 * dt * w2[i];
    velocities_dflow(wtemp, u_torque, w3);

    for(i=0;i<2;i++)
        wtemp[i] = x[i] + dt * w3[i];
    velocities_dflow(wtemp, u_torque, w4);

    for(i=0; i<2; i++)
        speeds[i] = x[i] + (dt/6.0)*(w1[i] + 2.0 * w2[i] + 2.0 * w3[i] + w4[i]);
}

SkidSteerDynamicModel::~SkidSteerDynamicModel()
{
    cout << "Destruindo instancia da classe SkidSteerDynamicModel." << endl;
}

#endif
