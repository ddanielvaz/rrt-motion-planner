#include <iostream>

#include "../geometry.h"
#include "draw.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
    double f[]={1.8, 4.0, 3.13, 0.0, 0.0};
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.25;
    char obstacles_file[]="../lasi_map.txt", pathfile[32], resultsfile[32],
    odomfile[32], title[]="Results";
    CarGeometry geom_car(width, height, body_length);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    Graphics fig(&geom_car, title);
    if(argc == 3){
        strcpy(pathfile, argv[1]);
        strcpy(odomfile, argv[2]);
    }
    else if(argc == 4)
    {
        strcpy(pathfile, argv[1]);
        strcpy(odomfile, argv[2]);
        strcpy(resultsfile, argv[3]);
        cout << resultsfile << endl;
    }
    else{
        strcpy(pathfile, "path.log");
        strcpy(odomfile, "odom.log");
        strcpy(resultsfile, "results.log");
    }
    //draw trajectory
//     fig.plot_obstacles(obstacles_file);
//     fig.plot_line_states(pathfile, c_blue);
//     fig.plot_line_states(odomfile, c_red);
//     fig.show(title);
    //draw trails
    fig.plot_obstacles(obstacles_file);
    fig.plot_trail_states(resultsfile, c_light_green);
    fig.plot_trail_states(pathfile, c_blue);
    fig.show(title);
    return 0;
}
