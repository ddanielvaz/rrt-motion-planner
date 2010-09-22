#include <iostream>

#include "geometry.h"
#include "draw.h"

using namespace std;

int main(int argc, char *argv[])
{
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    char obstacles_file[]="../resources/lasi_map.txt", pathfile[32], odomfile[32],
    title[]="Drawing Tool";
    CarGeometry geom_car(width, height, body_length);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    Graphics fig(&geom_car, obstacles_file, title);
    if(argc == 3){
        strcpy(pathfile, argv[1]);
        strcpy(odomfile, argv[2]);
    }
    else{
        strcpy(pathfile, "path.log");
        strcpy(odomfile, "odom.log");
    }
    //draw trajectory
    fig.plot_obstacles();
    fig.plot_line_states(pathfile, c_blue);
    fig.plot_line_states(odomfile, c_red);
    fig.show(title);
    return 0;
}
