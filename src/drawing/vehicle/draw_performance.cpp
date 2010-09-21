#include <iostream>

#include "draw.h"
#include "../geometry.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
    //baliza
    double f[]={4.40, 2.5, 0.0, 0.0, 0.0};
    //Manobra 1
    //double f[]={2.0, 3.7, 3.13, 0.0, 0.0};
    //Manobra 2
    //double f[]={5.8, 3.7, 0.0, 0.0, 0.0};
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.26;
    char obstacles_file[]="lasi_map.txt";
    char pathfile[32], rrtfile[32], temp[32];
    int index;
    ifstream statisticfile;
    if(argc>1)
        statisticfile.open(argv[1]);
    else
        statisticfile.open("all.log");
    CarGeometry geom_car(width, height, body_length);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
//     fig.plot_obstacles(obstacles_file);
//     fig.draw_initial_and_goal(q,f);
//     fig.plot_states(pathfile, c_blue);
//     fig.show();
    while(statisticfile.getline(temp, 32))
    {
        index = atoi(temp);
        snprintf(rrtfile, 32, "results%d.log", index);
        snprintf(pathfile, 32, "path%d.log", index);
        Graphics fig(&geom_car, pathfile);
        fig.plot_obstacles(obstacles_file);
        fig.plot_trail_states(rrtfile, c_light_green);
        fig.draw_initial_and_goal(q,f);
        fig.plot_trail_states(pathfile, c_blue);
        fig.show(pathfile);
    }
    return 0;
}
