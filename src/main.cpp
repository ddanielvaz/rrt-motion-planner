#include <iostream>

#include "draw.h"
#include "rrt.h"
#include "robots.h"
#include "utils.h"
#include "world.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={2.0, 6.5, 0.0, 0.0, 0.0};
    double f[]={12, 8.0, 0.0, 0.0, 0.0};
    double width = 2.5, height = 1.5, body_length = 2.0;
    char logfile[] = "results.log", obstacles_file[]="obstacles.txt",
         pathfile[] = "path.log";
    CarLikeModel veh(body_length);
    //SkidSteerModel veh;
    CarGeometry geom_car(width, height, body_length);
    World w(obstacles_file, &geom_car);
    RRT plan(q, f, 500, &veh, &w, logfile);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    plan.build();
    plan.close_logfile();
    plan.path_to_closest_goal();
    Graphics fig(&veh, &geom_car);
    fig.plot_obstacles(obstacles_file);
    fig.plot_states(logfile, c_light_green);
    fig.draw_initial_and_goal(q,f);
    fig.show();
    fig.plot_states(pathfile, c_blue);
    fig.show();
    return 0;
}
