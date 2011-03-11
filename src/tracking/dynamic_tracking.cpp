#include "PlayerTracking.hh"
#include "TrackingControl.hh"
#include "SkidSteerDynamicModel.hh"

#define SKID_STEER_DYNAMIC_N_STATES 5

int main(int argc, char *argv[])
{
    char pathfile[] = "path.log", ip[32];
    double xcir = 0.1;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 30.6, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.11, 10.0};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    SkidSteerDynamicModel p3at(motor, robot, speeds_limits, 5);
    PlayerTracking track(&p3at);
    if(argc > 1)
    {
        strcpy(ip, argv[1]);
//         track.SkidSteerControlBasedController(pathfile, ip);
        track.ProportionalController(pathfile, ip);
//         track.ControlByFierro(pathfile, ip);
//         track.NoControl(pathfile, ip);
//         track.Kanayama(pathfile, ip)

    }
    else
    {
        track.ProportionalController(pathfile, NULL);
//         track.SkidSteerControlBasedController(pathfile, NULL);
//         track.NoControl(pathfile, NULL);
//         track.Kanayama(pathfile, NULL);
    }
}
