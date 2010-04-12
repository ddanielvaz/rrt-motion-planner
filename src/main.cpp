#include <iostream>

#include "draw.h"
#include "geometry.h"
#include "rrt.h"
#include "robots.h"
#include "utils.h"
#include "world.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={5.0, 5.0, 0.0, 0.0, 0.0};
    double f[]={12, 8.0, 0.0, 0.0, 0.0};
    double width = 2.5, height = 1.5, body_length = 2.0;
    char logfile[] = "results.log", obstacles_file[]="obstacles.txt",
         pathfile[] = "path.log", logcontrol[]="controls.log";
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, 0.008, 0.122, 0.138, 0.1975, 0.4};
    SkidSteerDynamicModel veh(motor, robot, 5);
    //CarLikeModel veh(body_length, 3);
    //SkidSteerModel veh(3);
    CarGeometry geom_car(width, height, body_length);
    World w(obstacles_file, &geom_car);
    RRT plan(q, f, 800, &veh, &w, logfile, logcontrol);
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
