#ifndef __WORLD_MODULE_HH__
#define __WORLD_MODULE_HH__

#include <PQP/PQP.h>

#include "Geometry.hh"
#include "Constants.hh"

// #include <cmath>
// #include <iostream>
#include <cstdlib>
#include <fstream>

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
        int IsVehicleInCollision(double, double, double);
        CarGeometry *veh_geometry;
        EnvModel *env;
};

#endif
