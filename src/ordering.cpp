#include <iostream>
#include <fstream>

#include "utils.h"

using namespace std;

int main(int argc, char *argv[])
{
    char pathfile[32];
    char temp[1024], *ps, *nxt;
    double f[]={4.40, 2.5, 0.0, 0.0, 0.0};
    double state[3];
    double x, y, theta, v, w, aux, dt;
    double d, t, w_total, neg_t;
    ofstream statistic_fp("statistics.csv");
    // Parametro: distância final do ponto de destino.
    // Parametro: tempo total do percurso
    // Parametro: integração absoluta da velocidade angular w
    // Parametro: tempo em velocidade negativa
    for(int i=0; i<100; i++)
    {
        snprintf(pathfile, 32, "path%d.log", i);
        ifstream pathfp(pathfile);
        if(!pathfp.is_open())
        {
            cerr << "Arquivo " << pathfile << " nao encontrado." << endl;
            continue;
        }
        d = 0.0;
        t = 0.0;
        w_total = 0.0;
        neg_t = 0.0;
        while(pathfp.getline(temp, 1024))
        {
            x = strtod(temp, &ps);
            nxt = ps;
            y = strtod(nxt, &ps);
            nxt = ps;
            theta = strtod(nxt, &ps);
            nxt = ps;
            v = strtod(nxt, &ps);
            nxt = ps;
            w = strtod(nxt, &ps);
            nxt = ps;
            aux = strtod(nxt, &ps);
            nxt = ps;
            aux = strtod(nxt, &ps);
            nxt = ps;
            dt = strtod(nxt, NULL);
            w_total += fabs(w*dt);
            t += dt;
            if(v<0)
                neg_t += dt;
        }
        state[0] = x; state[1] = y; state[2] = theta;
        d = metric(state, f);
        statistic_fp << pathfile << ", " << d << ", " << t << ", " << w_total << ", " << neg_t << endl;
    }
    statistic_fp.close();
    return 0;
}
