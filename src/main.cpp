#include <iostream>

#include "simple_car.h"
#include "rrt.h"
#include "world.h"

using namespace std;



int main()
{
    double q[]={3.0, 3.0, 0.0, 0.0, 0.0};
    double f[]={12.0, 10.0, 0.0, 0.0, 0.0};
    ModelCar veh(q, 2.5, 2.0);
    World w;
    RRT plan(q, f, 500, &veh, &w);
    w.create_env_model();
    w.create_veh_model(&veh);
    plan.build();
    return 0;
}
