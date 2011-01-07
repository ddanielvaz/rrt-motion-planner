#include <iostream>

#include "Geometry.hh"
#include "RRT.hh"
#include "SkidSteerControlBased.hh"
#include "FierroControl.hh"
#include "World.hh"
#include "SpecificMeterP3AT.hh"

using namespace std;

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.0, 0.0};
//     Baliza
//     double f[]={4.40, 2.5, 0.1, 0.0, 0.0};
//     Manobra 1
//     double f[]={2.0, 3.7, 3.13, 0.0, 0.0};
//     Manobra 2
    double f[]={5.8, 3.8, 0.0, 0.0, 0.0};
//     Dimensoes para o pioneer 3at
    double width = 0.51, height = 0.493, body_length = 0.0;
    char obstacles_file[]="../resources/lasi.map";
//     char obstacles_file[]="../resources/grande.map";
//     char logfile[]="results.log";
//     char pathfile[]="path.log";
    char accel_file[]= "../resources/p3at.accel";
    
    double xcir = 0.01;
//     double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 30.6, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.11, 10.0};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    StateSampler ss;
    SpecificMeterP3AT dm(f);
//     DistanceMeter dm;
//     double constraints[] = {1.0, MAX_STEERING_ANGLE};
//     SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
//     veh.GenerateInputs("p3at.torques");
    SkidSteerControlBased veh(robot, speeds_limits, 5);
    struct timeval t_begin, t_end;
    double t0, t1;
    char statistic_file[] = "statistic.log";
    ofstream statistic_fp;
    statistic_fp.open(statistic_file);
    veh.SetPDTrajectoryControl();
    veh.GenerateInputs(accel_file);
    veh.trajectory_control->InitializeControllerWeights(1.0, 1.0, 1.0, 1.0);
    CarGeometry geom_car(width, height, body_length);
    World w(obstacles_file, &geom_car);
    for(int i=0; i<25; i++)
    {
        char pathfile[32], logfile[32];
        snprintf(logfile, 32, "results%d.log", i);
        gettimeofday(&t_begin, NULL);
        RRT plan(q, f, 20000, &veh, &w, &dm, &ss, logfile);
        //RRT plan(q, f, 10000, &veh, &w, logfile);
        plan.build();
        gettimeofday(&t_end, NULL);
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // tempo total;numero de tentativas;numero de nos adicionados;destino alcancado
        statistic_fp << t1-t0 << ";" << plan.trials << ";" << plan.added_nodes << ";" <<  plan.is_goal_reached << endl;
        plan.close_logfile();
        snprintf(pathfile, 32, "path%d.log",i);
        plan.path_to_closest_goal(pathfile);
    }
    statistic_fp.close();
    return 0;
}
