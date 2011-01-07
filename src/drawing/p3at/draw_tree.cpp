#include <iostream>

#include "Geometry.hh"
#include "RRT.hh"
#include "SkidSteerControlBased.hh"
#include "FierroControl.hh"
#include "World.hh"
#include "SpecificMeterP3AT.hh"

#include "Graphics.hh"

using namespace std;

int main(int argc, char *argv[])
{
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    double xcir = 0.008;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.4};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    char accel_file[]= "../resources/p3at.accel";
    char obstacles_file[]="../resources/lasi.map", pathfile[32], resultsfile[32],
    title[]="Drawing Tool";
    CarGeometry geom_car(width, height, body_length);
   SkidSteerControlBased veh(robot, speeds_limits, 5);
    veh.SetPDTrajectoryControl();
    veh.GenerateInputs(accel_file);
    veh.trajectory_control->InitializeControllerWeights(1.0, 1.0, 1.0, 1.0);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    Graphics fig(&geom_car, obstacles_file, title);
    if(argc == 3)
    {
        strcpy(resultsfile, argv[1]);
        strcpy(pathfile, argv[2]);
    }
    else{
        strcpy(pathfile, "path.log");
        strcpy(resultsfile, "results.log");
    }
    //draw rrt
    fig.plot_obstacles();
    fig.plot_tree(resultsfile, c_green, &veh);
    fig.plot_line_states(pathfile, c_red);
    fig.show(title);
    return 0;
}
