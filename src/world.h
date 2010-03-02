#ifndef _WORLD_MODULE_H_
#define _WORLD_MODULE_H_

#include <math.h>
#include <iostream>
using namespace std;

#include <PQP/PQP.h>

#include "simple_car.h"

PQP_REAL IDENTITY_MATRIX[3][3] = {{1.0, 0.0, 0.0},
                                  {0.0, 1.0, 0.0},
                                  {0.0, 0.0, 1.0}};
PQP_REAL ORIGIN[3] = {0.0, 0.0, 0.0};

class World
{
    public:
        World();
        ~World();
        void create_veh_model(ModelCar*);
        void create_env_model(void);
        bool is_vehicle_in_collision(PQP_REAL [3], PQP_REAL [][3]);
        double dim[2];
    private:
        PQP_Model env, veh;
};

World::World()
{
    cout << "Criando instancia da classe World." << endl;
    dim[0] = 20.0;
    dim[1] = 20.0;
}

World::~World()
{
    cout << "Destruindo instancia da classe World." << endl;
}

void World::create_env_model()
{
    //parelelo eixo x inferior
    PQP_REAL p0[3]={0,0,0}, p1[3]={0,0.1,0}, p2[3]={20,0.1,0};
    PQP_REAL p3[3]={20,0.1,0}, p4[3]={20,0,0}, p5[3]={0,0,0};
    //parelelo eixo y esquerda
    PQP_REAL p6[3]={0,0,0}, p7[3]={0.1,0,0}, p8[3]={0,20,0};
    PQP_REAL p9[3]={0,20,0}, p10[3]={0.1,20,0}, p11[3]={0.1,0,0};
    //parelelo eixo x superior
    PQP_REAL p12[3]={0,20,0}, p13[3]={0,20.1,0}, p14[3]={20,20,0};
    PQP_REAL p15[3]={20,20,0}, p16[3]={20,20.1,0}, p17[3]={0,20.1,0};
    //parelelo eixo y direita
    PQP_REAL p18[3]={20,20,0}, p19[3]={20.1,20,0}, p20[3]={20.1,0,0};
    PQP_REAL p21[3]={20.1,0,0}, p22[3]={20,0,0}, p23[3]={20,20,0};
    env.BeginModel();
    env.AddTri(p0, p1, p2, 0);
    env.AddTri(p3, p4, p5, 1);
    env.AddTri(p6, p7, p8, 2);
    env.AddTri(p9, p10, p11, 3);
    env.AddTri(p12, p13, p14, 4);
    env.AddTri(p15, p16, p17, 5);
    env.AddTri(p18, p19, p20, 6);
    env.AddTri(p21, p22, p23, 7);
    env.EndModel();
}

void World::create_veh_model(ModelCar *car)
{
    double w, l, x, y, *ini_pos;
    l = car->get_body_length();
    w = car->get_body_width();
    ini_pos = car->get_initial_position();
    x = ini_pos[STATE_X] + 0.2;
    y = ini_pos[STATE_Y] + 0.2;
    PQP_REAL p0[3]={x, y, 0}, p1[3]={x+l, y, 0}, p2[3]={x+l, y+w, 0};
    PQP_REAL p3[3]={x, y, 0}, p4[3]={x, y+w, 0}, p5[3]={x+l, y+w, 0};
    veh.BeginModel();
    veh.AddTri(p0, p1, p2, 0);
    veh.AddTri(p3, p4, p5, 1);
    veh.EndModel();    
}

bool World::is_vehicle_in_collision(PQP_REAL trans[3], PQP_REAL rot[][3])
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
}
/*
int i,j;
    PQP_Model m1, m2;
    PQP_REAL p1[3], p2[3], p3[3];
    PQP_REAL q1[3], q2[3], q3[3];
    PQP_REAL r1[3], r2[3], r3[3];

    PQP_REAL R1[3][3];
    PQP_REAL T1[3],T2[3];
    
    for(i=0; i<3; i++)
    {
        T1[i] = 0;
        T2[i] = 0;
        for(j=0; j<3; j++)
        {
            if (i != j)
                R1[i][j] = 0;
            else
                R1[i][j] = 1;
        }
    }

    // initialize the points
    //p1 = {3,5,7};
    cout << p1[0] << " " << p1[1] << " " << p1[2] << endl;
    p1[0] = 1; p1[1] = 1; p1[2] = 0;
    p2[0] = 1; p2[1] = 3; p2[2] = 0;
    p3[0] = 2; p3[1] = 1; p3[2] = 0;
    
    q1[0] = 1; q1[1] = 3; q1[2] = 0;
    q2[0] = 2; q2[1] = 3; q2[2] = 0;
    q3[0] = 2; q3[1] = 1; q3[2] = 0;

    r1[0] = 3; r1[1] = 2; r1[2] = 0;
    r2[0] = 3; r2[1] = 3; r2[2] = 0;
    r3[0] = 4; r3[1] = 3; r3[2] = 0;
    // add triangles that will belong to m1
    m1.BeginModel();
    m1.AddTri(p1, p2, p3, 0);
    m1.AddTri(q1, q2, q3, 1);
    m1.EndModel();

    m2.BeginModel();
    m2.AddTri(r1, r2, r3, 0);
    m2.EndModel();*/
#endif