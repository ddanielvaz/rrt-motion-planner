#ifndef _WORLD_MODULE_H_
#define _WORLD_MODULE_H_

#include <math.h>
#include <iostream>

#include <PQP/PQP.h>

#include "simple_car.h"

PQP_REAL IDENTITY_MATRIX[3][3] = {{1.0, 0.0, 0.0},
                                  {0.0, 1.0, 0.0},
                                  {0.0, 0.0, 1.0}};

PQP_REAL ORIGIN[3] = {0.0, 0.0, 0.0};

using namespace std;

class World
{
    public:
        World(ModelCar*);
        ~World();
        void create_veh_model(ModelCar*);
        void create_env_model(void);
        //bool is_vehicle_in_collision(PQP_REAL [3], PQP_REAL [][3]);
        bool is_vehicle_in_collision(double, double, double);
        double dim[2];
    private:
        PQP_Model env;
        double l, w, h; 
};

World::World(ModelCar* car)
{
    cout << "Criando instancia da classe World." << endl;
    l = car->get_between_axes_length();
    w = car->get_body_width();
    h = car->get_body_height();
    dim[0] = 20.0;
    dim[1] = 20.0;
}

World::~World()
{
    cout << "Destruindo instancia da classe World." << endl;
}

void World::create_env_model()
{
    ifstream obs("obstacles.txt");
    char temp[100], *ps, *nxt;
    double v[9];
    int i,j, tri_count=0;
    PQP_REAL p0[3], p1[3], p2[3];
    env.BeginModel();
    while(obs.getline(temp, 100))
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
        env.AddTri(p0, p1, p2, tri_count);
        tri_count++;
    }
    env.EndModel();
}

void World::create_veh_model(ModelCar *car)
{
    PQP_Model veh;
    double xo, yo, theta, *ini_pos;
    double xrt, yrt, xlt, ylt, xrb, yrb, xlb, ylb, x_aux, y_aux;
    double rot_mat[4];
    ini_pos = car->get_initial_position();
    xo = ini_pos[STATE_X];
    yo = ini_pos[STATE_Y];
    theta = ini_pos[STATE_THETA];
    rot_mat[0] = cos(theta);
    rot_mat[1] = -sin(theta);
    rot_mat[2] = sin(theta);
    rot_mat[3] = cos(theta);
    
    x_aux=xo + l + (w-l)/2.0;
    y_aux=yo + h/2.0;
    xrt = x_aux * rot_mat[0] + y_aux * rot_mat[1];
    yrt = x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    x_aux=xo - (w-l)/2.0;
    y_aux=yo + h/2.0;
    xlt = x_aux * rot_mat[0] + y_aux * rot_mat[1];
    ylt = x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    x_aux=xo - (w-l)/2.0;
    y_aux=yo - h/2.0;
    xlb=x_aux * rot_mat[0] + y_aux * rot_mat[1];
    ylb=x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    x_aux=xo + l + (w-l)/2.0;
    y_aux=yo - h/2.0;
    xrb=x_aux * rot_mat[0] + y_aux * rot_mat[1];
    yrb=x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    PQP_REAL p0[3]={xlt, ylt, 0}, p1[3]={xrt, yrt, 0}, p2[3]={xrb, yrb, 0};
    PQP_REAL p3[3]={xlt, ylt, 0}, p4[3]={xrb, yrb, 0}, p5[3]={xlb, ylb, 0};
    
    veh.BeginModel();
    veh.AddTri(p0, p1, p2, 0);
    veh.AddTri(p3, p4, p5, 1);
    veh.EndModel();    
}

/*bool World::is_vehicle_in_collision(PQP_REAL trans[3], PQP_REAL rot[][3])
{
    PQP_CollideResult cres;
    PQP_DistanceResult dres;
    double rel_err = 0.0, abs_err = 0.0, distance = 0.0;
    int colliding;
    PQP_Collide(&cres, IDENTITY_MATRIX, ORIGIN, &env, rot, trans, &veh);
    colliding = cres.Colliding();
    //cout << "Colliding env and car: " << colliding << endl;
    PQP_Distance(&dres, IDENTITY_MATRIX, ORIGIN, &env, rot, trans, &veh,
                 rel_err, abs_err);
    distance = dres.Distance();
    //cout << "Distance between env and car: " << distance << endl;
    if (colliding)
        return true;
    return false;
}*/

bool World::is_vehicle_in_collision(double x, double y, double theta)
{
    PQP_Model veh;
    double rot_mat[4], xrt, yrt, xlt, ylt, xrb, yrb, xlb, ylb, x_aux, y_aux;
    PQP_CollideResult cres;
    PQP_DistanceResult dres;
    double rel_err = 0.0, abs_err = 0.0, distance = 0.0;
    int colliding;
    rot_mat[0] = cos(theta);
    rot_mat[1] = -sin(theta);
    rot_mat[2] = sin(theta);
    rot_mat[3] = cos(theta);
    
    x_aux=x + l + (w-l)/2.0;
    y_aux=y + h/2.0;
    xrt = x_aux * rot_mat[0] + y_aux * rot_mat[1];
    yrt = x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    x_aux=x - (w-l)/2.0;
    y_aux=y + h/2.0;
    xlt = x_aux * rot_mat[0] + y_aux * rot_mat[1];
    ylt = x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    x_aux=x - (w-l)/2.0;
    y_aux=y - h/2.0;
    xlb=x_aux * rot_mat[0] + y_aux * rot_mat[1];
    ylb=x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    x_aux=x + l + (w-l)/2.0;
    y_aux=y - h/2.0;
    xrb=x_aux * rot_mat[0] + y_aux * rot_mat[1];
    yrb=x_aux * rot_mat[2] + y_aux * rot_mat[3];
    
    PQP_REAL p0[3]={xlt, ylt, 0}, p1[3]={xrt, yrt, 0}, p2[3]={xrb, yrb, 0};
    PQP_REAL p3[3]={xlt, ylt, 0}, p4[3]={xrb, yrb, 0}, p5[3]={xlb, ylb, 0};
    
    veh.BeginModel();
    veh.AddTri(p0, p1, p2, 0);
    veh.AddTri(p3, p4, p5, 1);
    veh.EndModel();
    PQP_Collide(&cres, IDENTITY_MATRIX, ORIGIN, &env, IDENTITY_MATRIX, ORIGIN, &veh);
    colliding = cres.Colliding();
    //cout << "Colliding env and car: " << colliding << endl;
    PQP_Distance(&dres, IDENTITY_MATRIX, ORIGIN, &env, IDENTITY_MATRIX, ORIGIN, &veh,
                 rel_err, abs_err);
    distance = dres.Distance();
    //cout << "Distance between env and car: " << distance << endl;
    if (colliding)
        return true;
    return false;
}
#endif