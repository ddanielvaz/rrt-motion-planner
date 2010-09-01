/********************************************************************
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ********************************************************************/

/***************************************************************************
 * Desc: Classe Robot
 * Authors: Daniel Vaz
 * email: danielvaz (at) gmail (dot) com
 ***************************************************************************/

#include <iostream>
#include <libplayerc++/playerc++.h>

#include "position2d_proxy.h"

using namespace std;
using namespace PlayerCc;

class Robot{
    public:
        // Atributos
        PlayerClient *client;
        // Métodos
        Robot (char*); //Construtor
        ~Robot(); //Destrutor
        ProxyPosition *navigator;
};

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

