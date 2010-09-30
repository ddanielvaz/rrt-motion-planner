#include "Control.hh"
#include "PlayerTracking.hh"

#define SKID_STEER_DYNAMIC_N_STATES 5

int main(int argc, char *argv[])
{
    char pathfile[] = "path.log", ip[32];
    Tracking track(SKID_STEER_DYNAMIC_N_STATES);
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
        track.control_01(pathfile, NULL);
//         track.control_02(pathfile, NULL);
        //track.control(pathfile, NULL);
        //track.control_kanayama(pathfile, NULL);
        //track.control_kanayama_delay(pathfile, NULL);
//         track.no_control(pathfile, NULL);
    }
}
