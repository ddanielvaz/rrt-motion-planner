#include "../utils.h"
#include "../robots.h"

int main(int argc, char *argv[])
{
    double xcir = 0.008;
    double motor[] = {0.0230, 0.0230, 38.3, 0.71};
    double robot[] = {0.413, 40, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.4};
    double speeds_limits[] = {0.3, 2.44};
    double q[]={1.0, 1.5, 0.0, 0.5, 0.0};
    SkidSteerDynamicModel veh(motor, robot, speeds_limits, 5);
    double aux[veh.n_states], temp[veh.n_states], control_in[2]={0,0};
    memcpy(aux, q, sizeof(double) * veh.n_states);
    for (double it=0.0; it<20.0; it+=DELTA_T)
    {
        cout << "v: " << aux[3] << " w: " << aux[4] << endl << endl;
        veh.EstimateNewState(DELTA_T, aux, control_in, temp);
        temp[STATE_THETA] = normalize_angle(temp[STATE_THETA]);
        memcpy(aux, temp, sizeof(double) * veh.n_states);
    }
//     cout << "x: " << aux[0] << " y: " << aux[1] << " angle: " << aux[2]
//          << " v: " << aux[3] << " w: " << aux[4] << endl;
    cout << "v_f: " << aux[3] << " w_f: " << aux[4] << endl << endl;
}
