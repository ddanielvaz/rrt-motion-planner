#include "PlayerTracking.hh"
#include "TrackingControl.hh"
#include "SkidSteerControlBased.hh"

#define SKID_STEER_DYNAMIC_N_STATES 5

int main(int argc, char *argv[])
{
    char pathfile[] = "path.log", ip[32];
    double xcir = 0.1;
    double robot[] = {0.413, 30.6, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.11, 10.0};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    SkidSteerControlBased p3at(robot, speeds_limits, 5);
    PlayerTracking track(&p3at);
    p3at.trajectory_control->InitializeControllerWeights(4.0, 20.0, 10.0, 0.01);
    if(argc > 1)
    {
        strcpy(ip, argv[1]);
        track.control_01(pathfile, ip);
//         track.control_02(pathfile, ip);
        //track.control(pathfile, ip);
        //track.control_kanayama(pathfile, ip);
        //track.control_kanayama_delay(pathfile, ip);
        //track.no_control(pathfile, ip);
    }
    else
    {
//         track.ControlByFierro(pathfile, NULL);
        track.NoControl(pathfile, NULL);
//         track.control_02(pathfile, NULL);
        //track.control(pathfile, NULL);
        //track.control_kanayama(pathfile, NULL);
        //track.control_kanayama_delay(pathfile, NULL);
//         track.no_control(pathfile, NULL);
    }
}
