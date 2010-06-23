#include <iostream>
#include <fstream>

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
    double aux[veh.n_states], temp[veh.n_states];
    double control_in[2]={0,0};
    double max_torque=10.0, torque_step = 0.5;
    double max_lin_speed = 0.5, speed_step = 0.05;
    double inst_accel, inst_accel_rot, total_time=1.0;
    char filename[32];
    ofstream fp;
    for(double speed=-max_lin_speed; speed<=max_lin_speed; speed=speed+speed_step)
    {
        snprintf(filename, 32, "torque_v%.2f.csv", speed);
        fp.open(filename);
        fp << "v_i,Torque Right,Torque Left,v_f,dv" << endl;
        q[3] = speed;
        cout << "speed: " << speed << endl;
        for(double torque=-max_torque; torque<=max_torque; torque=torque+torque_step)
        {
            memcpy(aux, q, sizeof(double) * veh.n_states);
            control_in[0] = torque;
            control_in[1] = torque;
            for (double it=0.0; it<total_time; it+=DELTA_T)
            {
                veh.EstimateNewState(DELTA_T, aux, control_in, temp);
                temp[STATE_THETA] = normalize_angle(temp[STATE_THETA]);
                memcpy(aux, temp, sizeof(double) * veh.n_states);
            }
            inst_accel = (aux[3] - q[3])/total_time;
            inst_accel_rot = (aux[4] - q[4])/total_time;
            if(fabs(inst_accel) < MAX_LIN_ACCEL && fabs(inst_accel_rot) < MAX_STEER_ACCEL)
            {
                fp << q[3] << "," << control_in[0] << "," << control_in[1]
                   << "," << aux[3] << "," << inst_accel << endl;
                cout << "[final] x: " << aux[0] << " y: " << aux[1] << " theta: " << aux[2] << endl;
                cout << "[initial] v: " << q[3] << " w: " << q[4] << endl;
                cout << "[final] v: " << aux[3] << " w: " << aux[4] << endl;
                cout << "absolute acceleration: " << inst_accel << endl << endl;
            }
        }
        fp.close();
    }
//     cout << "x: " << aux[0] << " y: " << aux[1] << " angle: " << aux[2]
//          << " v: " << aux[3] << " w: " << aux[4] << endl;
}
