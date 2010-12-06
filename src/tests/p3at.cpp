#include <iostream>

#include "Geometry.hh"
#include "RRT.hh"
#include "SkidSteerControlBased.hh"
#include "FierroControl.hh"
#include "World.hh"
#include "SpecificMeterP3AT.hh"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
//     Baliza
//     double f[]={4.40, 2.5, 0.1, 0.0, 0.0};
//     Manobra 1
//     double f[]={2.0, 3.7, 3.13, 0.0, 0.0};
//     Manobra 2
    double f[]={5.8, 3.8, 0.0, 0.0, 0.0};
//     Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    char obstacles_file[]="../resources/lasi.map";
//     char obstacles_file[]="../resources/grande.map";
    char logfile[]="results.log";
    char pathfile[]="path.log";
    char accel_file[]= "../resources/p3at.accel";
    double xcir = 0.01;
//     double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 30.6, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.11, 10.0};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    StateSampler ss;
    SpecificMeterP3AT dm(f);
//     DistanceMeter dm;
//     double constraints[] = {1.0, MAX_STEERING_ANGLE};
//     SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
//     veh.GenerateInputs("p3at.torques");
    SkidSteerControlBased veh(robot, speeds_limits, 5);
    veh.SetPDTrajectoryControl();
//     veh.SetFierroTrajectoryControl();
    veh.GenerateInputs(accel_file);
    veh.trajectory_control->InitializeControllerWeights(1.0, 3.0, 1.0, 4.0);
//     veh.trajectory_control->InitializeControllerWeights(1.0, 3.0, 1.0, 4.0);
//     CarLikeModel veh(body_length, constraints, 5);
//     veh.GenerateInputs("../resources/carlike.accel");
    CarGeometry geom_car(width, height, body_length);
    World w(obstacles_file, &geom_car);
    RRT plan(q, f, 20000, &veh, &w, &dm, &ss, logfile);
    plan.build();
    plan.close_logfile();
    plan.path_to_closest_goal(pathfile);
    return 0;
}
