#include <iostream>

#include "../geometry.h"
#include "../robots.h"
#include "../constants.h"
#include "draw.h"

using namespace std;

int main(int argc, char *argv[])
{
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    double xcir = 0.008;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.4};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    /** Tipos de Robos */
    SkidSteerControlBased veh(robot, speeds_limits, 5);
//     SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
    char obstacles_file[]="lasi_map.txt", pathfile[32], resultsfile[32],
    title[]="Drawing Tool";
    CarGeometry geom_car(width, height, body_length);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    Graphics fig(&geom_car, title);
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
    fig.plot_obstacles(obstacles_file);
    fig.plot_tree(resultsfile, c_blue, &veh);
    fig.plot_line_states(pathfile, c_green);
    fig.show(title);
    return 0;
}
