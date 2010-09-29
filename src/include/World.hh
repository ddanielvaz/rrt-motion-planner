#ifndef __WORLD_MODULE_H__
#define __WORLD_MODULE_H__

#include <PQP/PQP.h>

#include "Geometry.hh"
#include "Constants.hh"

// #include <cmath>
// #include <iostream>
#include <cstdlib>
#include <fstream>

PQP_REAL IDENTITY_MATRIX[3][3] = {{1.0, 0.0, 0.0},
                                  {0.0, 1.0, 0.0},
                                  {0.0, 0.0, 1.0}};

PQP_REAL ORIGIN[3] = {0.0, 0.0, 0.0};

class EnvModel
{
    public:
        EnvModel(char *);
        ~EnvModel();
        PQP_Model obstacles;
        double dim[2];
};

class World
{
    public:
        World(char *, CarGeometry *);
        ~World();
        int IsVehicleInSafePosition(double, double, double);
        CarGeometry *veh_geometry;
        EnvModel *env;
};

#endif
