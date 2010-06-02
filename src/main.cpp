#include <iostream>

#include "geometry.h"
#include "rrt.h"
#include "robots.h"
#include "utils.h"
#include "world.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
    // Baliza
    //double f[]={4.40, 2.5, 0.0, 0.0, 0.0};
    // Manobra 1
    double f[]={2.0, 3.7, 3.13, 0.0, 0.0};
    // Manobra 2
    //double f[]={5.8, 3.7, 0.0, 0.0, 0.0};
    //Dimensoes para um veiculo
    //double width = 2.5, height = 1.5, body_length = 2.0;
    //Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    char obstacles_file[]="lasi_map.txt", logcontrol[]="controls.log";
    double xcir = 0.008;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.4};
    double speeds_limits[] = {0.3, 2.44};
    SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
    //PioneerCarLikeModel veh(motor, robot, speeds_limits, 5);
    //CarLikeModel veh(body_length, 3);
    //SkidSteerModel veh(3, xcir);
    CarGeometry geom_car(width, height, body_length);
    World w(obstacles_file, &geom_car);
    for(int i=0; i<25; i++)
    {
        char pathfile[32], logfile[32];
        snprintf(logfile, 32, "results%d.log", i);
        RRT plan(q, f, 5000, &veh, &w, logfile, logcontrol);
        plan.build();
        plan.close_logfile();
        snprintf(pathfile, 32, "path%d.log",i);
        plan.path_to_closest_goal(pathfile);
    }
    return 0;
}
