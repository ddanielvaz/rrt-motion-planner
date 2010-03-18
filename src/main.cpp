#include <iostream>

#include "draw.h"
#include "rrt.h"
#include "simple_car.h"
#include "utils.h"
#include "world.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={5.0, 5.0, 0.0, 0.0, 0.0};
    double f[]={15.0, 15.0, 0.0, 0.0, 0.0};
    char logfile[] = "results.log", obstacles_file[]="obstacles.txt",
         pathfile[] = "path.log";
    ModelCar veh(q, 2.0, 1.0, 1.5);
    World w(obstacles_file, &veh);
    RRT plan(q, f, 1000, &veh, &w, logfile);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    plan.build();
    plan.close_logfile();
    plan.path_finder();
    Graphics fig(&veh, &w);
    fig.read_and_plot(logfile, c_light_green);
    fig.draw_initial_and_goal(q,f);
    fig.show();
    fig.read_and_plot(pathfile, c_blue);
    fig.show();
    
    return 0;
}
