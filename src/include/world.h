#ifndef _WORLD_MODULE_H_
#define _WORLD_MODULE_H_

#include <cmath>
#include <iostream>

#include <PQP/PQP.h>

#include "robots.h"

PQP_REAL IDENTITY_MATRIX[3][3] = {{1.0, 0.0, 0.0},
                                  {0.0, 1.0, 0.0},
                                  {0.0, 0.0, 1.0}};

PQP_REAL ORIGIN[3] = {0.0, 0.0, 0.0};

using namespace std;

class EnvModel
{
    public:
        EnvModel(char *);
        ~EnvModel();
        PQP_Model obstacles;
        double dim[2];
};

EnvModel::EnvModel(char *filename)
{
    cout << "Criando instancia da classe EnvModel" << endl;
    ifstream fp(filename);
    char temp[100], *ps, *nxt;
    double v[9];
    int i,j, tri_count=0;
    PQP_REAL p0[3], p1[3], p2[3];
    fp.getline(temp, 100);
    dim[0] = strtod(temp, &ps);
    nxt = ps;
    dim[1] = strtod(nxt, NULL);
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
        obstacles.AddTri(p0, p1, p2, tri_count);
        tri_count++;
    }
    obstacles.EndModel();
    fp.close();
}

EnvModel::~EnvModel()
{
    cout << "Destruindo instancia da classe EnvModel" << endl;
}

class World
{
    public:
        World(char *, CarGeometry *);
        ~World();
        int IsVehicleInSafePosition(double, double, double);
        CarGeometry *veh_geometry;
        EnvModel *env;
};

World::World(char *envfilename, CarGeometry* car_geom)
{
    cout << "Criando instancia da classe World." << endl;
    env = new EnvModel(envfilename);
    veh_geometry = car_geom;
}

int World::IsVehicleInSafePosition(double x, double y, double theta)
{
//     PQP_CollideResult cres;
    PQP_DistanceResult dres;
    PQP_Model *col_model_env, carro;
    double rel_err = 0.0, abs_err = 0.0, distance = 0.0;
//     int colliding;
    PQP_REAL p0[3], p1[3], p2[3], p3[3], p4[3], p5[3];
    veh_geometry->SetVerticesPosition(x, y, theta);
    p0[2] = p1[2] = p2[2] = p3[2] = p4[2] = p5[2] = 0.0;
    // Primeiro triangulo
    p0[0] = veh_geometry->xlt; p0[1] = veh_geometry->ylt;
    p1[0] = veh_geometry->xrb; p1[1] = veh_geometry->yrb;
    p2[0] = veh_geometry->xlb; p2[1] = veh_geometry->ylb;
    // Segundo triangulo
    p3[0] = veh_geometry->xlt; p3[1] = veh_geometry->ylt;
    p4[0] = veh_geometry->xrb; p4[1] = veh_geometry->yrb;
    p5[0] = veh_geometry->xrt; p5[1] = veh_geometry->yrt;
    carro.BeginModel();
    carro.AddTri(p0, p1, p2, 0);
    carro.AddTri(p3, p4, p5, 1);
    carro.EndModel();

    col_model_env = &(env->obstacles);
/*    PQP_Collide(&cres, IDENTITY_MATRIX, ORIGIN, col_model_env,
                IDENTITY_MATRIX, ORIGIN, &carro);
    colliding = cres.Colliding();*/
    //cout << "Colliding env and car: " << colliding << endl;
    PQP_Distance(&dres, IDENTITY_MATRIX, ORIGIN, col_model_env,
                 IDENTITY_MATRIX, ORIGIN, &carro, rel_err, abs_err);
    distance = dres.Distance();
    if(distance < COLLISION_TOLERANCE)
        return COLLIDED;
    //cout << "Distance between env and car: " << distance << endl;
    return !COLLIDED;
}

World::~World()
{
    cout << "Destruindo instancia da classe World." << endl;
}

#endif