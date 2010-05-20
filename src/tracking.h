#include <iostream>
#include <fstream>

#include "robots.h"
#include "robot_proxy.h"

using namespace std;

player_pose2d_t calculate_speed(player_pose2d_t dq, double theta)
{
    const double xcir=0.008, xcir2_1=(xcir*xcir)+1.0;
    player_pose2d_t vel;
    vel.px = dq.px * cos(theta) + dq.py * sin(theta);
    vel.pa = dq.px*(-xcir*sin(theta)/xcir2_1) + dq.py*(xcir*cos(theta)/xcir2_1)
             + dq.pa/xcir2_1;
    return vel;
}

player_pose2d_t calculate_dq(player_pose2d_t speed, double theta)
{
    const double xcir=0.008;
    player_pose2d_t dq;
    dq.px = speed.px * cos(theta) - speed.pa * xcir* sin(theta);
    dq.py = speed.px * sin(theta) + speed.pa * xcir * cos(theta);
    dq.pa = speed.pa;
    return dq;
}

class Tracking
{
    public:
        Tracking(RobotModel*);
        void control_kanayama(char *, char *);
        void control(char *, char *);
        RobotModel *robot;
};

Tracking::Tracking(RobotModel *r)
{
    cout << "Criando instancia da classe Tracking." << endl;
    robot = r;
}

void Tracking::control_kanayama(char *log, char *ip)
{
    int n = robot->n_states;
    //double it, states[n], aux1[n], aux2[n], control[2];
    double data[n];
    char temp[100], *nxt, *ps;
    ifstream path_fp(log);
    player_pose2d_t path_log_pos, old_path_pos, odom_pos, curr_pos, old_pos;
    ofstream odom_fp("odom.log");
    ofstream data_fp("data.log");
    double vx_robot, vy_robot, va_robot;
    double vx_path, va_path;
    double vx_e, va_e, x_e, y_e, angle_e;
    double x_diff, y_diff, angle_diff;
    double vx_control, va_control;
    double Kx=2, Ky=4, Ktheta=4;
    Robot r0(ip);
    // Mudando modo como os dados são lidos do servidor.
    // Desse modo garante que as leituras serão sempre as mais novas.
    try
    {
        r0.client->SetDataMode(PLAYER_DATAMODE_PULL);
        r0.client->SetReplaceRule(true, -1, -1, -1);
    }
    catch (PlayerCc::PlayerError e)
    {
        cerr << e << endl;
        return;
    }
    r0.client->Read();
    path_fp.getline(temp, 100);
    data[0] = strtod(temp, &ps);
    nxt = ps;
    for (int i=1; i<n; i++)
    {
        data[i] = strtod(nxt, &ps);
        nxt = ps;
    }
    path_log_pos.px = data[0];
    path_log_pos.py = -data[1];
    path_log_pos.pa = -data[2];
    old_path_pos = path_log_pos;
    vx_path = data[3];
    va_path = -data[4];
    r0.odom->SetOdomPos(path_log_pos);
    r0.odom->SetMotorStatus(true);
    r0.client->Read();
    old_pos = r0.odom->GetPose();
    
    while(path_fp.getline(temp, 100))
    {
        // Requisita dados dos sensores.
        r0.client->Read();
        curr_pos = odom_pos = r0.odom->GetPose();
        vx_robot = r0.odom->GetXSpeed();
        vy_robot = r0.odom->GetYSpeed();
        va_robot = r0.odom->GetYawSpeed();
        vx_e = (vx_path - vx_robot);
        va_e = (va_path - va_robot);
        // Calculando erro de posição da trajetória planejada com leitura do odometro
        x_diff = (path_log_pos.px - curr_pos.px);
        y_diff = (path_log_pos.py - curr_pos.py);
        angle_diff = normalize_angle(path_log_pos.pa - curr_pos.pa);
        // Transformando para origem em curr_pos.px, curr_pos.py, curr_pos.pa
        x_e = cos(curr_pos.pa) * x_diff + sin(curr_pos.pa) * y_diff;
        y_e = cos(curr_pos.pa) * y_diff - sin(curr_pos.pa) * x_diff;
        angle_e = angle_diff;
        vx_control = limit_speed(vx_path*cos(angle_e) + Kx * x_e, 0.4);
        va_control = limit_speed(va_path + vx_path*(Ky*y_e + Ktheta * sin(angle_e)), 0.6);
/*
        // Preenchendo estrutura auxiliar
        aux_dq.px = x_diff;
        aux_dq.py = y_diff;
        aux_dq.pa = angle_diff;
        // Transformando para coordenas referentes ao angulo local
        speed_error = calculate_speed(aux_dq, curr_pos.pa);
        p_trans_error = kp_trans * speed_error.px;
        p_rot_error = kp_rot * speed_error.pa;
        vx_control = limit_speed(vx_path + p_trans_error, 0.3);
        va_control = limit_speed(va_path + p_rot_error, 0.6);
*/
        // Logando informações
        data_fp << "x_path: " << path_log_pos.px << " y_path: "
                << path_log_pos.py << " angle_path: " << path_log_pos.pa << endl;
        data_fp << "vx_path: " << vx_path << " va_path: " << va_path << endl;
        data_fp << "x_odom: " << odom_pos.px << " y_odom: " << odom_pos.py
                << " angle_odom: " << odom_pos.pa << endl;
        data_fp << "vx_robot: " << vx_robot << " vy_robot: " << vy_robot
                << " va_robot: " << va_robot << endl;
        data_fp << "x_e: " << x_e << " y_e: " << y_e << " angle_e:" << angle_e << endl;
        data_fp << "vx_e: " << vx_e << " va_e: " << va_e << endl;
        data_fp << "vx_control: " << vx_control << " va_control: " << va_control
                << endl << endl;
        // Fim do Log
        data[0] = strtod(temp, &ps);
        nxt = ps;
        for (int i=1; i<n; i++)
        {
            data[i] = strtod(nxt, &ps);
            nxt = ps;
        }
        old_path_pos = path_log_pos;
        path_log_pos.px = data[0];
        // Corrigindo sinais de y, angle e velocidade angular
        path_log_pos.py = -data[1];
        path_log_pos.pa = -data[2];
        vx_path = data[3];
        va_path = -data[4];
        old_pos = curr_pos;
        odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
        r0.odom->AdjustSpeed(vx_control, va_control);
        usleep(INTEGRATION_TIME * 1e6);
    }
    path_fp.close();
    r0.odom->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    odom_pos = r0.odom->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
    odom_fp.close();
    data_fp.close();
    r0.odom->AdjustSpeed(0.0, 0.0);
}

void Tracking::control(char *log, char *ip)
{
    int n = robot->n_states;
    //double it, states[n], aux1[n], aux2[n], control[2];
    double data[n];
    char temp[100], *nxt, *ps;
    ifstream path_fp(log);
    player_pose2d_t path_log_pos, old_path_pos, odom_pos, curr_pos, old_pos;
    player_pose2d_t speed_error, aux_dq;
    ofstream odom_fp("odom.log");
    ofstream data_fp("data.log");
    double vx_robot, vy_robot, va_robot;
    double vx_path, va_path;
    double x_diff, y_diff, angle_diff;
    double vx_control, va_control;
    double wp = 1;
    double kp_trans=1.0*wp, kp_rot=0.8*wp;
    double p_trans_error, p_rot_error;
    Robot r0(ip);
    // Mudando modo como os dados são lidos do servidor.
    // Desse modo garante que as leituras serão sempre as mais novas.
    try
    {
        r0.client->SetDataMode(PLAYER_DATAMODE_PULL);
        r0.client->SetReplaceRule(true, -1, -1, -1);
    }
    catch (PlayerCc::PlayerError e)
    {
        cerr << e << endl;
        return;
    }
    r0.client->Read();
    path_fp.getline(temp, 100);
    data[0] = strtod(temp, &ps);
    nxt = ps;
    for (int i=1; i<n; i++)
    {
        data[i] = strtod(nxt, &ps);
        nxt = ps;
    }
    path_log_pos.px = data[0];
    path_log_pos.py = -data[1];
    path_log_pos.pa = -data[2];
    old_path_pos = path_log_pos;
    vx_path = data[3];
    va_path = -data[4];
    r0.odom->SetOdomPos(path_log_pos);
    r0.odom->SetMotorStatus(true);
    r0.client->Read();
    old_pos = r0.odom->GetPose();
    
    while(path_fp.getline(temp, 100))
    {
        // Requisita dados dos sensores.
        r0.client->Read();
        curr_pos = odom_pos = r0.odom->GetPose();
        vx_robot = r0.odom->GetXSpeed();
        vy_robot = r0.odom->GetYSpeed();
        va_robot = r0.odom->GetYawSpeed();
        // Calculando erro de posição da trajetória planejada com leitura do odometro
        x_diff = (path_log_pos.px - curr_pos.px);
        y_diff = (path_log_pos.py - curr_pos.py);
        angle_diff = normalize_angle(path_log_pos.pa - curr_pos.pa);
        // Preenchendo estrutura auxiliar
        aux_dq.px = x_diff;
        aux_dq.py = y_diff;
        aux_dq.pa = angle_diff;
        // Transformando para coordenas referentes ao angulo local
        speed_error = calculate_speed(aux_dq, curr_pos.pa);
        p_trans_error = kp_trans * speed_error.px;
        p_rot_error = kp_rot * speed_error.pa;
        vx_control = limit_speed(vx_path + p_trans_error, 0.3);
        va_control = limit_speed(va_path + p_rot_error, 0.6);

        // Logando informações
        data_fp << "x_path: " << path_log_pos.px << " y_path: "
                << path_log_pos.py << " angle_path: " << path_log_pos.pa << endl;
        data_fp << "vx_path: " << vx_path << " va_path: " << va_path << endl;
        data_fp << "x_odom: " << odom_pos.px << " y_odom: " << odom_pos.py
                << " angle_odom: " << odom_pos.pa << endl;
        data_fp << "vx_robot: " << vx_robot << " vy_robot: " << vy_robot
                << " va_robot: " << va_robot << endl;
        data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << angle_diff << endl;
        data_fp << "speed_error_x: " << speed_error.px << " speed_error_a: " << speed_error.pa << endl;
        data_fp << "vx_control: " << vx_control << " va_control: " << va_control
                << endl << endl;
        // Fim do Log
        data[0] = strtod(temp, &ps);
        nxt = ps;
        for (int i=1; i<n; i++)
        {
            data[i] = strtod(nxt, &ps);
            nxt = ps;
        }
        old_path_pos = path_log_pos;
        path_log_pos.px = data[0];
        // Corrigindo sinais de y, angle e velocidade angular
        path_log_pos.py = -data[1];
        path_log_pos.pa = -data[2];
        vx_path = data[3];
        va_path = -data[4];
        old_pos = curr_pos;
        odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
        r0.odom->AdjustSpeed(vx_control, va_control);
        usleep(INTEGRATION_TIME * 1e6);
    }
    path_fp.close();
    r0.odom->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    odom_pos = r0.odom->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
    odom_fp.close();
    data_fp.close();
    r0.odom->AdjustSpeed(0.0, 0.0);
}

  /*  
  
  Dado o torque do modelo dinamico, estima as velocidades a partir do metodo
  EstimateNewState.
  
    states[0] = strtod(temp, &ps);
    nxt = ps;
    for (int i=1; i<robot->n_states; i++)
    {
        states[i] = strtod(nxt, &ps);
        nxt = ps;
    }
    control[0] = strtod(nxt, &ps);
    nxt = ps;
    control[1] = strtod(nxt, NULL);
    

    memcpy(aux1, states, sizeof(double) * robot->n_states);
    for (it=0.0; it<INTEGRATION_TIME; it+=DELTA_T)
    {
        robot->EstimateNewState(DELTA_T, aux1, control, aux2);
        memcpy(aux1, aux2, sizeof(double) * robot->n_states);
    }
    cout << aux1[3] << " " << aux1[4] << endl;
    */

