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

#endif
