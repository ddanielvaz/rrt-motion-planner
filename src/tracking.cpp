#include "tracking.h"
#include "robots.h"

int main(int argc, char *argv[])
{
    char pathfile[] = "path.log", ip[32];
    double xcir = 0.008;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.4};
    double speeds_limits[] = {0.3, 2.44};
    SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
    Tracking track(&veh);
    if(argc > 1){
        strcpy(ip, argv[1]);
        track.control(pathfile, ip);
        //track.control_kanayama(pathfile, ip);
    }
    else
        track.control(pathfile, NULL);
        //track.control_kanayama(pathfile, NULL);
}