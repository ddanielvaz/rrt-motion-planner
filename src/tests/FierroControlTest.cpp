#include "SkidSteerControlBased.hh"
#include "FierroControl.hh"

int main(int argc, char *argv[])
{
    double q[]={1.0, 1.5, 0.0, 0.2, 0.3};
    double u[2]={0.0, 0.0};
//     Dimensoes para o pioneer 3at
    double xcir = 0.1;
    double robot[] = {0.413, 30.6, 0.043, 0.506, xcir, 0.138, 0.122, 0.1975, 0.11, 10.0};
    double speeds_limits[] = {MAX_LIN_SPEED, MAX_ROT_SPEED};
    char accel_file[]= "../resources/p3at.accel";
    double curr_vel[2], new_vel[2], curr_pos[3], new_pos[3];
    double ideal_states[5], inputs[2], computed_torques[2];
    double new_vel_tracked[2], new_pos_tracked[3], tracked_states[5];

    SkidSteerControlBased veh(robot, speeds_limits, 5);
    double k = 4.0;
    veh.trajectory_control->InitializeControllerWeights(k, k, k, 0.1);
    veh.GenerateInputs(accel_file);

    // Preenchendo variáveis auxiliares para obtenção do novo estado através de
    // entrada de controle e integração numérica.
    // Copiando as três primeiras posições do vetor do espaço de estados
    // referentes a: (x,y,psi)
    memcpy(curr_pos, q, sizeof(double) * 3);
    // Copiando as duas últimas posições do vetor do espaço de estados
    // referentes a: (v,w)
    memcpy(curr_vel, q+3, sizeof(double) * 2);
    // Estima velocidade futura a partir da velocidade atual e das acelerações
    // linear e angular fornecidas.
    veh.EstimateVelocities(curr_vel, u, new_vel);
    // Copiando velocidade estimada para vetor de estados ideais
    memcpy(ideal_states+3, new_vel, sizeof(double) * 2);
    //Estima posição futura a partir da posição atual e da velocidade futura
    // estimada anteriormente.
    veh.EstimatePosition(curr_pos, new_vel, new_pos);
    // Copiando posição estimada para vetor de estados ideais
    memcpy(ideal_states, new_pos, sizeof(double) * 3);
    //XXX: talvez criar um método SimulateRobot para realizar as etapas acima.
    veh.trajectory_control->run(q, ideal_states, inputs);
    veh.EstimateTorque(q, inputs, computed_torques);
    veh.EstimateVelocitiesFromTorque(q, computed_torques, new_vel_tracked);
    // Copiando velocidade estimada para vetor de estados acompanhados
    memcpy(tracked_states+3, new_vel_tracked, sizeof(double) * 2);
    //Estima posição futura a partir da posição atual e da velocidade de
    // acompanhamento estimada anteriormente.
    veh.EstimatePosition(curr_pos, new_vel_tracked, new_pos_tracked);
    memcpy(tracked_states, new_pos_tracked, sizeof(double) * 3);
    cout << "Current Vx: " << curr_vel[0] << " Current W: " << curr_vel[1] << endl;
    cout << "Ideal ref Vx: " << new_vel[0] << " Ideal ref W: " << new_vel[1] << endl;
    cout << "Torque_l: " << computed_torques[TORQUE_L] << " Torque_r: " << computed_torques[TORQUE_R] << endl;
    cout << "Tracked Vx: " << new_vel_tracked[0] << " Tracked W: " << new_vel_tracked[1] << endl << endl;
//     cout << "x: " << new_pos_tracked[0] << " y: " << new_pos_tracked[1] << " psi: " << new_pos_tracked[2] << endl << endl;
    return 0;
}