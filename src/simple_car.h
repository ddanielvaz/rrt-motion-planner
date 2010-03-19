#ifndef _MODEL_SIMPLE_CAR_H_
#define _MODEL_SIMPLE_CAR_H_

#include <math.h>
#include <string.h>
#include <iostream>

#include "utils.h"

using namespace std;

class ModelCar
{
    private:
        double m_one_over_bodyLength;
        double initial[N_STATES], curr_speed, curr_steering, length, width, height;
    public:
        ModelCar(double *, double, double, double);
        ~ModelCar();
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
        double get_between_axes_length();
        double get_body_width();
        double get_body_height();
        double* get_initial_position();
};

ModelCar::ModelCar(double *position, double w, double h, double bodyLength)
{
    cout << "Criando instancia da classe ModelCar" << endl;
    initial[STATE_X] = position[0];
    initial[STATE_Y] = position[1];
    initial[STATE_THETA] = position[2];
    length = bodyLength;
    width = w;
    height = h;
    m_one_over_bodyLength = 1.0/bodyLength;
    curr_speed = 0.0;
    curr_steering = 0.0;
}

ModelCar::~ModelCar()
{
    cout << "Destruindo instancia da classe ModelCar" << endl;
}

void ModelCar::dflow(const double *x, const double *u, double *dx)
{
    dx[STATE_X] = u[CONTROL_VELOCITY] * cos(x[STATE_THETA]);
    dx[STATE_Y] = u[CONTROL_VELOCITY] * sin(x[STATE_THETA]);
    dx[STATE_THETA] = u[CONTROL_VELOCITY] * m_one_over_bodyLength * tan(u[CONTROL_STEERING_ANGLE]);
}

void ModelCar::EstimateNewState(const double dt, const double *x,
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

double ModelCar::get_between_axes_length()
{
    return length;
}

double ModelCar::get_body_width()
{
    return width;
}

double ModelCar::get_body_height()
{
    return height;
}

double* ModelCar::get_initial_position()
{
    return initial;
}

#endif
