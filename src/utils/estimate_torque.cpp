#include "../utils.h"
#include "../robots.h"

int main(int argc, char *argv[])
{
    double xcir = 0.008;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.4};
    double speeds_limits[] = {0.3, 2.44};
    double q[]={0.0, 0.0, 0.0, 0.0, 0.0};
    SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
    double aux[veh.n_states], temp[veh.n_states], control_in[2]={0,0};
    for(int i=0;i<10;i++)
    {
        control_in[0] = -(7+(double)i/10.0);
        control_in[1] = -(7+(double)i/10.0);
        cout << "Torques: " << i << endl;
        memcpy(aux, q, sizeof(double) * veh.n_states);
        for (double it=0.0; it<1.0; it+=DELTA_T)
        {
            veh.EstimateNewState(DELTA_T, aux, control_in, temp);
            temp[STATE_THETA] = normalize_angle(temp[STATE_THETA]);
            memcpy(aux, temp, sizeof(double) * veh.n_states);
        }
        cout << "v: " << aux[3] << " w: " << aux[4] << endl;
    }
//     cout << "x: " << aux[0] << " y: " << aux[1] << " angle: " << aux[2]
//          << " v: " << aux[3] << " w: " << aux[4] << endl;
}
