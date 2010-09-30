#ifndef __ROBOT_PROXY_HH__
#define __ROBOT_PROXY_HH__

#include <libplayerc++/playerc++.h>

#include "position2d_proxy.h"

#include <iostream>

using namespace std;
using namespace PlayerCc;

class Robot{
    public:
        // Atributos
        PlayerClient *client;
        ProxyPosition *navigator;
        // Métodos
        Robot (char*); //Construtor
        ~Robot(); //Destrutor
};

#endif
