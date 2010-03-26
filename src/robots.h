#ifndef _ROBOTS_MODEL_H_
#define _ROBOTS_MODEL_H_

#include <math.h>
#include <string.h>
#include <iostream>

#include "utils.h"

using namespace std;

class ModelCar
{
    private:
        double m_one_over_bodyLength;
    public:
        ModelCar(double);
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
};

ModelCar::ModelCar(double bodyLength)
{
    cout << "Criando instancia da classe ModelCar" << endl;
    m_one_over_bodyLength = 1.0/bodyLength;
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

class CarGeometry{
    public:
        CarGeometry(double, double, double);
        ~CarGeometry();
        double xrt, yrt, xrb, yrb, xlt, ylt, xlb, ylb;
        double get_between_axes_length(void);
        double get_body_width(void);
        double get_body_height(void);
        void position(double, double, double);
    private:
        double w, h, l;
};

CarGeometry::CarGeometry(double width, double height, double length)
{
    cout << "Criando instancia da classe CarGeometry." << endl;
    w = width;
    h = height;
    l = length;
}
/**
 * A partir de um ponto de rotação (x,y) e de um ângulo de rotação theta,
 * gera os vertices de um retangulo de largura w, altura h.
 * @param x coordenada x, do ponto em torno do qual o retangulo será rotacionado.
 * @param y coordenada y, do ponto em torno do qual o retangulo será rotacionado.
 * @param theta ângulo de rotação.
*/
void CarGeometry::position(double x, double y, double theta)
{
    double rot_mat[4], xc, yc, aux_x, aux_y;
    rot_mat[3] = rot_mat[0] = cos(theta);
    rot_mat[2] = sin(theta);
    rot_mat[1] = -rot_mat[2];

    xc=x + (w+l)/2.0;
    yc=y + h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrt = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrt = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x - (w-l)/2.0;
    yc=y + h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xlt = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    ylt = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x - (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xlb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    ylb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x + (w+l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;
}

double CarGeometry::get_between_axes_length()
{
    return l;
}

double CarGeometry::get_body_width()
{
    return w;
}

double CarGeometry::get_body_height()
{
    return h;
}

CarGeometry::~CarGeometry()
{
    cout << "Destruindo instancia da classe CarGeometry" << endl;
}

#endif
