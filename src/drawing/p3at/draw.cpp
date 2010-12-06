#include <iostream>

#include "Geometry.hh"
#include "Graphics.hh"

using namespace std;

int main(int argc, char *argv[])
{
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    char obstacles_file[]="../resources/lasi.map";
    char pathfile[32], odomfile[32];
    char title[]="Drawing Tool";
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
    double f[]={5.8, 3.8, 0.0, 0.0, 0.0};
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
    //draw trails
    fig.plot_obstacles();
    fig.plot_trail_states(pathfile, c_light_green);
    fig.plot_trail_states(odomfile, c_blue);
    fig.draw_initial_and_goal(q,f);
    fig.show(title);
    return 0;
}
