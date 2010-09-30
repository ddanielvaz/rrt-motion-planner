#ifndef __PLAYER_TRACKING_HH__
#define __PLAYER_TRACKING_HH__

#include <libplayerc++/playerc++.h>

#include <iostream>
#include <fstream>

using namespace PlayerCc;
using namespace std;

player_pose2d_t calculate_speed(player_pose2d_t dq, double theta);
player_pose2d_t calculate_dq(player_pose2d_t speed, double theta);

class Tracking
{
    public:
        Tracking(int);
        void control_kanayama(char *, char *);
        void control_kanayama_delay(char *, char *);
        void control(char *, char *);
        void control_01(char *, char *);
        void control_02(char *, char *);
        void no_control(char *, char *);
        int n_states;
};

#endif
