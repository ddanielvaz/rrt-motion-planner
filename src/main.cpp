#include <iostream>

#include "draw.h"
#include "rrt.h"
#include "simple_car.h"
#include "utils.h"
#include "world.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={2.0, 6.0, 0.0, 0.0, 0.0};
    double f[]={11.5, 8.5, 0.0, 0.0, 0.0};
    char logfile[] = "results.log", obstacles_file[]="obstacles.txt",
         pathfile[] = "path.log";
    ModelCar veh(q, 3.0, 1.5, 2.0);
    World w(obstacles_file, &veh);
    RRT plan(q, f, 500, &veh, &w, logfile);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    plan.build();
    plan.close_logfile();
    plan.path_to_closest_goal();
    Graphics fig(&veh);
    fig.plot_obstacles(obstacles_file);
    fig.plot_states(logfile, c_light_green);
    fig.draw_initial_and_goal(q,f);
    fig.show();
    fig.plot_states(pathfile, c_blue);
    fig.show();
    return 0;
}
