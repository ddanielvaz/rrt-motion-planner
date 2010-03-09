#include <iostream>

#include "simple_car.h"
#include "rrt.h"
#include "world.h"
#include "draw.h"

using namespace std;




int main(int argc, char *argv[])
{
    double q[]={5.0, 10.0, 0.0, 0.0, 0.0};
    double f[]={30.0, 3.0, 0.0, 0.0, 0.0};
    char logfile[] = "results.log";
    ModelCar veh(q, 2.0, 1.0, 1.5);
    World w(&veh);
    RRT plan(q, f, 38, &veh, &w, logfile);
    cvInitSystem(argc, argv);
    setlocale(LC_NUMERIC, "C");
    w.create_env_model();
    w.create_veh_model(&veh);
    plan.build();
    plan.close_logfile();
    Graphics fig(logfile, &veh);
    fig.plot();
    fig.show();
    return 0;
}
