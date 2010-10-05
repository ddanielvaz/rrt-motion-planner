#ifndef __PLAYER_TRACKING_HH__
#define __PLAYER_TRACKING_HH__

#define ODOM_LOG_FILE "odom.log"
#define DATA_LOG_FILE "data.log"

#include "RobotModel.hh"
#include "Pioneer3ATState.hh"

#include <libplayerc++/playerc++.h>

#include <iostream>
#include <fstream>

using namespace PlayerCc;
using namespace std;

player_pose2d_t calculate_speed(player_pose2d_t dq, double theta);
player_pose2d_t calculate_dq(player_pose2d_t speed, double theta);

class PlayerTracking
{
    public:
        PlayerTracking(RobotModel*);
        void control_kanayama(char *, char *);
        void control_kanayama_delay(char *, char *);
        void control(char *, char *);
        void control_01(char *, char *);
        void control_02(char *, char *);
        void no_control(char *, char *);
        void ControlByFierro(const char *, const char *);
        void ParseLog(const char *, Pioneer3ATState *);
        RobotModel* robot_model;
};

#endif
