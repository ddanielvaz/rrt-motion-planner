#include <iostream>

#include "simple_car.h"
#include "rrt.h"
#include "world.h"
#include "draw.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={5.0, 5.0, 0.0, 0.0, 0.0};
    double f[]={16.0, 7.0, 0.45, 0.0, 0.0};
    char logfile[] = "results.log", obstacles_file[]="obstacles.txt",
          pathfile[] = "path.log";
    ModelCar veh(q, 2.0, 1.0, 1.5);
    World w(obstacles_file, &veh);
    RRT plan(q, f, 500, &veh, &w, logfile);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    plan.build();
    plan.close_logfile();
    plan.path_finder();
    Graphics fig(logfile, &veh, &w);
    fig.read_and_plot();
    fig.show();
    Graphics fig1(pathfile, &veh, &w);
    fig1.read_and_plot();
    fig1.show();
    return 0;
}
