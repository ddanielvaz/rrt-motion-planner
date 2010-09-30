#include "RobotProxy.hh"

Robot::Robot(char *ip){
    cout << "Criando instancia da classe Robot." << endl;
    if(ip)
        client = new PlayerClient(ip, 6665);
    else
        client = new PlayerClient("localhost", 6665);
    navigator = new ProxyPosition(client);
}

Robot::~Robot()
{
    cout << "Destruindo instancia da classe." << endl;
    delete navigator;
    delete client;
}
