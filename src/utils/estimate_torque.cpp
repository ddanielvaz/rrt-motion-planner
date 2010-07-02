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
    char filename[32], torque_file[32];
    ofstream fp, torque_fp;
    strncpy(torque_file, "lin_torques.csv", 32);
    torque_fp.open(torque_file);
    strncpy(filename, "lineares.csv", 32);
    fp.open(filename);
    fp << "vi,vf,dv,wi,wf,dw,Torque Right,Torque Left" << endl;
    for(double speed=-max_lin_speed; speed<=max_lin_speed; speed=speed+speed_step)
    {
        q[3] = speed;
//         cout << "speed: " << speed << endl;
//         torque_fp << speed << ";";
        for(double torque=-max_torque; torque<=max_torque; torque=torque+torque_step)
        {
            memcpy(aux, q, sizeof(double) * veh.n_states);
            control_in[TORQUE_R] = torque;
            control_in[TORQUE_L] = torque;
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
                if(fabs(aux[3]) <= MAX_LIN_SPEED && fabs(aux[4]) <= MAX_ROT_SPEED)
                {
                    fp << q[3] << "," << aux[3] << "," << inst_accel << "," << q[4]
                    << "," << aux[4] << "," << inst_accel_rot << ","
                    << control_in[0] << "," << control_in[1] << endl;
                    torque_fp << control_in[0] << "," << control_in[1] << ";";
//                     cout << "[final] x: " << aux[0] << " y: " << aux[1] << " theta: " << aux[2] << endl;
//                     cout << "[initial] v: " << q[3] << " w: " << q[4] << endl;
//                     cout << "[final] v: " << aux[3] << " w: " << aux[4] << endl;
//                     cout << "absolute acceleration: " << inst_accel << endl << endl;
                }
            }
        }
        torque_fp << endl;
    }
    fp.close();
    torque_fp.close();
    strncpy(torque_file, "torques_to_right.csv", 32);
    strncpy(filename, "curve_to_right.csv", 32);
    fp.open(filename);
    torque_fp.open(torque_file);
    fp << "vi,vf,dv,wi,wf,dw,Torque Right,Torque Left" << endl;
    for(double speed=-max_lin_speed; speed<=max_lin_speed; speed=speed+speed_step)
    {
        q[3] = speed;
        q[4] = speed;
        cout << "speed: " << speed << endl;
        for(double torque=-max_torque; torque<=max_torque; torque=torque+torque_step)
        {
            memcpy(aux, q, sizeof(double) * veh.n_states);
            control_in[TORQUE_R] = torque;
            control_in[TORQUE_L] = torque+2.0;
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
                if(fabs(aux[3]) <= MAX_LIN_SPEED && fabs(aux[4]) <= MAX_ROT_SPEED)
                {
                    fp << q[3] << "," << aux[3] << "," << inst_accel << "," << q[4]
                    << "," << aux[4] << "," << inst_accel_rot << ","
                    << control_in[0] << "," << control_in[1] << endl;
                    torque_fp << control_in[0] << "," << control_in[1] << ";";
//                     cout << "[final] x: " << aux[0] << " y: " << aux[1] << " theta: " << aux[2] << endl;
//                     cout << "[initial] v: " << q[3] << " w: " << q[4] << endl;
//                     cout << "[final] v: " << aux[3] << " w: " << aux[4] << endl;
//                     cout << "absolute acceleration: " << inst_accel_rot << endl << endl;
                }
            }
        }
        torque_fp << endl;
    }
    fp.close();
    torque_fp.close();
    strncpy(filename, "curve_to_left.csv", 32);
    fp.open(filename);
    strncpy(torque_file, "torques_to_left.csv", 32);
    torque_fp.open(torque_file);
    fp << "vi,vf,dv,wi,wf,dw,Torque Right,Torque Left" << endl;
    for(double speed=-max_lin_speed; speed<=max_lin_speed; speed=speed+speed_step)
    {
        q[3] = speed;
        q[4] = speed;
        cout << "speed: " << speed << endl;
        for(double torque=-max_torque; torque<=max_torque; torque=torque+torque_step)
        {
            memcpy(aux, q, sizeof(double) * veh.n_states);
            control_in[TORQUE_R] = torque+2.0;
            control_in[TORQUE_L] = torque;
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
                if(fabs(aux[3]) <= MAX_LIN_SPEED && fabs(aux[4]) <= MAX_ROT_SPEED)
                {
                    fp << q[3] << "," << aux[3] << "," << inst_accel << "," << q[4]
                    << "," << aux[4] << "," << inst_accel_rot << ","
                    << control_in[0] << "," << control_in[1] << endl;
                    torque_fp << control_in[0] << "," << control_in[1] << ";";
//                     cout << "[final] x: " << aux[0] << " y: " << aux[1] << " theta: " << aux[2] << endl;
//                     cout << "[initial] v: " << q[3] << " w: " << q[4] << endl;
//                     cout << "[final] v: " << aux[3] << " w: " << aux[4] << endl;
//                     cout << "absolute acceleration: " << inst_accel_rot << endl << endl;
                }
            }
        }
        torque_fp << endl;
    }
    fp.close();
//     cout << "x: " << aux[0] << " y: " << aux[1] << " angle: " << aux[2]
//          << " v: " << aux[3] << " w: " << aux[4] << endl;
}
