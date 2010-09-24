#include <iostream>

#include "geometry.h"
#include "rrt.h"
#include "robots.h"
#include "utils.h"
#include "world.h"

using namespace std;

// #define TORQUE_LOGFILE "p3at.torques"
// #define ACCEL_LOGFILE "p3at.accel"
// #define CARLIKE_LOGFILE "carlike.accel"

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
    // Baliza
//     double f[]={4.40, 2.5, 0.1, 0.0, 0.0};
    // Manobra 1
//     double f[]={2.0, 3.7, 3.13, 0.0, 0.0};
    // Manobra 2
    double f[]={5.8, 3.7, 0.0, 0.0, 0.0};
// Pontos de referencia para o mapa grande
//     double q[]={2.0, 2.0, 0.0, 0.0, 0.0};
//     double f[]={16.0, 3.0, 0.0, 0.0, 0.0};
//     double f[]={9.0, 2.0, 0.0, 0.0, 0.0};
//     double f[]={16.0, 17.0, M_PI, 0.0, 0.0};
//     double f[]={21.0, 9.0, 0.0, 0.0, 0.0};
// Manobras para veiculo no mapa grande modificado para Baliza
//     double q[]={1.0, 3.5, 0.0, 0.0, 0.0};
//     double f[]={6.5, 1.5, 0.0, 0.0, 0.0};
    //Dimensoes para um veiculo
//     double width = 2.5, height = 1.5, body_length = 2.0;
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    char obstacles_file[]="../resources/lasi.map", logfile[]="results.log",
         pathfile[]="path.log";
    double xcir = 0.01;
//     double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.1};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
//     double constraints[] = {1.0, MAX_STEERING_ANGLE};
//     SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
//     veh.GenerateInputs("p3at.torques");
    SkidSteerControlBased veh(robot, speeds_limits, 5);
    veh.GenerateInputs("../resources/p3at.accel");
//     CarLikeModel veh(body_length, constraints, 5);
//     veh.GenerateInputs("../resources/carlike.accel");
    CarGeometry geom_car(width, height, body_length);
    World w(obstacles_file, &geom_car);
    RRT plan(q, f, 10000, &veh, &w, logfile);
    plan.build();
    plan.close_logfile();
    plan.path_to_closest_goal(pathfile);
    return 0;
}
