#include <iostream>

#include "simple_car.h"
#include "rrt.h"
#include "world.h"
#include "draw.h"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={5.0, 5.0, 0.0, 0.0, 0.0};
    double f[]={15.0, 15.0, 0.0, 0.0, 0.0};
    char logfile[] = "results.log", obstacles_file[]="obstacles.txt";
    
    ModelCar veh(q, 2.0, 1.0, 1.5);
    World w(obstacles_file, &veh);
    RRT plan(q, f, 500, &veh, &w, logfile);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    plan.build();
    plan.close_logfile();
    Graphics fig(logfile, &veh, &w);
    fig.plot();
    fig.show();
    return 0;
}
