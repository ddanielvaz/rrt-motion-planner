#include "Position2DProxy.hh"

ProxyPosition::ProxyPosition(PlayerClient *client) : Position2dProxy(client, 0)
{
    cout << "Criando instancia da classe ProxyPosition." << endl;
}

ProxyPosition::~ProxyPosition(void)
{
    cout << "Destruindo instancia da classe ProxyPosition." << endl;
}

/**
Ajusta a velocidade dos motores.
@param fwd this is the forward speed.
@param turn this is the turning speed.
*/
void ProxyPosition::AdjustSpeed(double fwd, double turn)
{
    this->SetSpeed(fwd, turn);
}

void ProxyPosition::SetMotorStatus(bool st)
{
    this->SetMotorEnable(st);
}

void ProxyPosition::SetOdomPos(player_pose2d_t pos)
{
    this->SetOdometry(pos.px, pos.py, pos.pa);
}

player_pose2d_t ProxyPosition::GetPose(void)
{
    player_pose2d_t pos;
    pos.px = this->GetXPos();
    pos.py = this->GetYPos();
    pos.pa = this->GetYaw();
    return pos;
}
