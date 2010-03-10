#ifndef _WORLD_MODULE_H_
#define _WORLD_MODULE_H_

#include <math.h>
#include <iostream>
#include <vector>

#include <PQP/PQP.h>
#include <cv.h>
#include <highgui.h>

#include "simple_car.h"

PQP_REAL IDENTITY_MATRIX[3][3] = {{1.0, 0.0, 0.0},
                                  {0.0, 1.0, 0.0},
                                  {0.0, 0.0, 1.0}};

PQP_REAL ORIGIN[3] = {0.0, 0.0, 0.0};

using namespace std;

class EnvModel
{
    public:
        EnvModel(char *);
        void draw(IplImage *);
        void plot_obstacles(IplImage *);
        ~EnvModel();
        PQP_Model obstacles;
        vector<CvPoint3D64f> triangs;
};

EnvModel::EnvModel(char *filename)
{
    cout << "Criando instancia da classe EnvModel" << endl;
    ifstream fp(filename);
    char temp[100], *ps, *nxt;
    double v[9];
    int i,j, tri_count=0;
    PQP_REAL p0[3], p1[3], p2[3];
    obstacles.BeginModel();
    while(fp.getline(temp, 100))
    {
        v[0] = strtod(temp, &ps);
        nxt = ps;
        for (i=1; i<9; i++)
        {
            v[i] = strtod(nxt, &ps);
            nxt = ps;
        }
        for(i=0, j=0; j<3; i++,j++)
            p0[j]=v[i];
        for(j=0; j<3; j++,i++)
            p1[j]=v[i];
        for(j=0; j<3; j++,i++)
            p2[j]=v[i];
        for(i=0;i<9;i=i+3)
            triangs.push_back(cvPoint3D64f(v[i], v[i+1], v[i+2]));
        obstacles.AddTri(p0, p1, p2, tri_count);
        tri_count++;
    }
    obstacles.EndModel();
    fp.close();
}

void EnvModel::plot_obstacles(IplImage *img)
{
    CvPoint triang[3];
    CvPoint *fig[1];
    int npts=3, ncurves=1;
    CvScalar red = CV_RGB(255,0,0);
    for (vector<CvPoint3D64f>::iterator i = triangs.begin(); i != triangs.end(); ){
        triang[0] = cvPoint(10.0 * i->x,10.0 * i->y);
        ++i;
        triang[1] = cvPoint(10.0 * i->x,10.0 * i->y);
        ++i;
        triang[2] = cvPoint(10.0 * i->x,10.0 * i->y);
        ++i;
        fig[0] = triang;
        cvPolyLine(img, fig, &npts, ncurves, 1, red);
    }
}

EnvModel::~EnvModel()
{
    cout << "Destruindo instancia da classe EnvModel" << endl;
}

class CarroModel
{
    public:
        CarroModel(ModelCar *, double, double, double);
        PQP_Model veiculo;
        ~CarroModel();        
};


CarroModel::CarroModel(ModelCar *car, double x, double y, double theta)
{
    //cout << "Criando instancia da classe Carro." << endl;
    double xrt, yrt, xlt, ylt, xlb, ylb, xrb, yrb, xc, yc, aux_x, aux_y, l, h, w;
    double rot_mat[4];
    PQP_REAL p0[3], p1[3], p2[3], p3[3], p4[3], p5[3];
    l = car->get_between_axes_length();
    h = car->get_body_width();
    w = car->get_body_height();
    rot_mat[3] = rot_mat[0] = cos(theta);
    rot_mat[2] = sin(theta);
    rot_mat[1] = -rot_mat[2];

    xc=x + l + (w-l)/2.0;
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

    xc=x + l + (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;
    
    p0[2] = p1[2] = p2[2] = p3[2] = p4[2] = p5[2] = 0.0;
    // Primeiro triangulo
    p0[0] = xlt; p0[1] = ylt;
    p1[0] = xrb; p1[1] = yrb;
    p2[0] = xlb; p2[1] = ylb;
    // Segundo triangulo
    p3[0] = xlt; p3[1] = ylt;
    p4[0] = xrb; p4[1] = yrb;
    p5[0] = xrt; p5[1] = yrt;
    veiculo.BeginModel();
    veiculo.AddTri(p0, p1, p2, 0);
    veiculo.AddTri(p3, p4, p5, 1);
    veiculo.EndModel();
}

CarroModel::~CarroModel()
{
    //cout << "Destruindo instancia da classe Carro." << endl;
}

class World
{
    public:
        World(char *, ModelCar *);
        ~World();
        bool is_vehicle_in_collision(double, double, double);
        void create_vehicle(double, double, double, PQP_Model *);
        double dim[2];
        ModelCar *veh_state_model;
        EnvModel *env;
};

World::World(char *envfilename, ModelCar* car)
{
    cout << "Criando instancia da classe World." << endl;
    env = new EnvModel(envfilename);
    dim[0] = 20.0;
    dim[1] = 20.0;
    veh_state_model = car;
}

bool World::is_vehicle_in_collision(double x, double y, double theta)
{
    PQP_CollideResult cres;
//    PQP_DistanceResult dres;
    PQP_Model *col_model_env, carro;
    
  //  double rel_err = 0.0, abs_err = 0.0, distance = 0.0;
    int colliding;
    
   /* 
    double xrt, yrt, xlt, ylt, xlb, ylb, xrb, yrb, xc, yc, aux_x, aux_y, l, h, w;
    double rot_mat[4];
    PQP_REAL p0[3], p1[3], p2[3], p3[3], p4[3], p5[3];
    l = veh_state_model->get_between_axes_length();
    h = veh_state_model->get_body_width();
    w = veh_state_model->get_body_height();
    rot_mat[3] = rot_mat[0] = cos(theta);
    rot_mat[2] = sin(theta);
    rot_mat[1] = -rot_mat[2];

    xc=x + l + (w-l)/2.0;
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

    xc=x + l + (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;
    
    p0[2] = p1[2] = p2[2] = p3[2] = p4[2] = p5[2] = 0.0;
    // Primeiro triangulo
    p0[0] = xlt; p0[1] = ylt;
    p1[0] = xrb; p1[1] = yrb;
    p2[0] = xlb; p2[1] = ylb;
    // Segundo triangulo
    p3[0] = xlt; p3[1] = ylt;
    p4[0] = xrb; p4[1] = yrb;
    p5[0] = xrt; p5[1] = yrt;
    carro.BeginModel();
    carro.AddTri(p0, p1, p2, 0);
    carro.AddTri(p3, p4, p5, 1);
    carro.EndModel();
    */
    
    create_vehicle(x, y, theta, &carro);
    col_model_env = &(env->obstacles);
    PQP_Collide(&cres, IDENTITY_MATRIX, ORIGIN, col_model_env,
                IDENTITY_MATRIX, ORIGIN, &carro);
    colliding = cres.Colliding();
    //cout << "Colliding env and car: " << colliding << endl;
    /*PQP_Distance(&dres, IDENTITY_MATRIX, ORIGIN, col_model_env,
                 IDENTITY_MATRIX, ORIGIN, &carro,
                 rel_err, abs_err);
    distance = dres.Distance();*/
    //cout << "Distance between env and car: " << distance << endl;
    if (colliding)
        return true;
    return false;
}

void World::create_vehicle(double x, double y, double theta, PQP_Model *veh)
{
    double xrt, yrt, xlt, ylt, xlb, ylb, xrb, yrb, xc, yc, aux_x, aux_y, l, h, w;
    double rot_mat[4];
    PQP_REAL p0[3], p1[3], p2[3], p3[3], p4[3], p5[3];
    l = veh_state_model->get_between_axes_length();
    h = veh_state_model->get_body_width();
    w = veh_state_model->get_body_height();
    rot_mat[3] = rot_mat[0] = cos(theta);
    rot_mat[2] = sin(theta);
    rot_mat[1] = -rot_mat[2];

    xc=x + l + (w-l)/2.0;
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

    xc=x + l + (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;
    
    p0[2] = p1[2] = p2[2] = p3[2] = p4[2] = p5[2] = 0.0;
    // Primeiro triangulo
    p0[0] = xlt; p0[1] = ylt;
    p1[0] = xrb; p1[1] = yrb;
    p2[0] = xlb; p2[1] = ylb;
    // Segundo triangulo
    p3[0] = xlt; p3[1] = ylt;
    p4[0] = xrb; p4[1] = yrb;
    p5[0] = xrt; p5[1] = yrt;
    veh->BeginModel();
    veh->AddTri(p0, p1, p2, 0);
    veh->AddTri(p3, p4, p5, 1);
    veh->EndModel();
}

World::~World()
{
    cout << "Destruindo instancia da classe World." << endl;
}

#endif