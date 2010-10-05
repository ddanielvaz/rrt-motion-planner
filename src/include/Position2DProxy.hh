#ifndef __POSITION_2D_PROXY_HH__
#define __POSITION_2D_PROXY_HH__

#include "Pioneer3ATState.hh"

#include <libplayerc++/playerc++.h>

#include <iostream>

using namespace std;
using namespace PlayerCc;

class ProxyPosition : public Position2dProxy
{
    public:
        ProxyPosition(PlayerClient *client);
        ~ProxyPosition(void);
        void AdjustSpeed(double fwd, double turn);
        void SetMotorStatus(bool st);
        void SetOdomPos(player_pose2d_t pos);
        player_pose2d_t GetPose(void);
        Pioneer3ATState GetRobotState(void);
};

#endif
