#include "Constants.hh"
#include "MathFunctions.hh"
#include "PlayerTracking.hh"
#include "RobotProxy.hh"
#include "SkidSteerControlBased.hh"
#include "TrackingControl.hh"

/** Funções auxiliares */

/**
Esta função auxiliar, implementa a transformação número (2) para estimar a
velocidade do robô P3AT.
--------------------------------------------------------------------------------
dq   => diferença entre estados (referência e atual)
eta  => vetor velocidade (linear e angular)
S(q) => Matriz que rege a cinemática do robô Pioneer3AT
--------------------------------------------------------------------------------
(1) dq = S(q) * eta
então:
(2) eta = S^-1(q) * dq

Obs: Como S(q) não é quadrada é calculada a pseudo-inversa.
*/
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

/** Declaração da Classe PlayerTracking */

PlayerTracking::PlayerTracking(RobotModel *r)
{
    cout << "Criando instancia da classe PlayerTracking." << endl;
    robot_model = r;
}


void PlayerTracking::control_kanayama_delay(char *log, char *ip)
{
    double data[robot_model->n_states];
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
    double Kx=2, Ky=1, Ktheta=2;
    // Delay
    struct timeval t_begin, t_end;
    double t0, t1;
    // End Delay
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
    for (int i=1; i<robot_model->n_states; i++)
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
    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    r0.client->Read();
    old_pos = r0.navigator->GetPose();
    
    while(path_fp.getline(temp, 100))
    {
        gettimeofday(&t_begin, NULL);
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // Requisita dados dos sensores.
        r0.client->Read();
        curr_pos = odom_pos = r0.navigator->GetPose();
        vx_robot = r0.navigator->GetXSpeed();
        vy_robot = r0.navigator->GetYSpeed();
        va_robot = r0.navigator->GetYawSpeed();
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
        va_control = va_path + vx_path*(Ky*y_e + Ktheta * sin(angle_e));

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
        for (int i=1; i<robot_model->n_states; i++)
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
        gettimeofday(&t_end, NULL);
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        r0.navigator->AdjustSpeed(vx_control, va_control);
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0)*1e6);
    }
    path_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    odom_pos = r0.navigator->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
    odom_fp.close();
    data_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
}

void PlayerTracking::control_01(char *log, char *ip)
{
    //double it, states[n], aux1[n], aux2[n], control[2];
    double data[robot_model->n_states];
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
    double kp_trans=0.2*wp, kp_rot=1.5*wp;
    double p_trans_error, p_rot_error;
    Robot r0(ip);
    // Delay
    struct timeval t_begin, t_end;
    double t0, t1;
    // End Delay
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
    for (int i=1; i<robot_model->n_states; i++)
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
    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    r0.client->Read();
    old_pos = r0.navigator->GetPose();
    
    for(int i=0;i<10;i++)
        r0.client->Read();
    
    while(path_fp.getline(temp, 100))
    {
        gettimeofday(&t_begin, NULL);
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // Requisita dados dos sensores.
        r0.client->Read();
        curr_pos = odom_pos = r0.navigator->GetPose();
        vx_robot = r0.navigator->GetXSpeed();
        vy_robot = r0.navigator->GetYSpeed();
        va_robot = r0.navigator->GetYawSpeed();
        // Calculando erro de posição da trajetória planejada com leitura do odometro
        x_diff = (old_path_pos.px - curr_pos.px);
        y_diff = (old_path_pos.py - curr_pos.py);
        angle_diff = normalize_angle(old_path_pos.pa - curr_pos.pa);
        // Preenchendo estrutura auxiliar
        aux_dq.px = x_diff;
        aux_dq.py = y_diff;
        aux_dq.pa = angle_diff;
        // Transformando para coordenas referentes ao angulo local
        speed_error = calculate_speed(aux_dq, curr_pos.pa);
        p_trans_error = kp_trans * speed_error.px;
        p_rot_error = kp_rot * speed_error.pa;
        vx_control = limit_speed(vx_path + p_trans_error, 0.7);
        va_control = va_path + p_rot_error;

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
        for (int i=1; i<robot_model->n_states; i++)
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
        gettimeofday(&t_end, NULL);
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        r0.navigator->AdjustSpeed(vx_control, va_control);
        data_fp << "PROCESSING TIME: " << (t1-t0) << endl;
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0) * 1e6);
    }
    while(fabs(x_diff) > 0.05 || fabs(angle_diff) > 0.05)
    {
        vx_path = 0.0;
        va_path = 0.0;
        gettimeofday(&t_begin, NULL);
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // Requisita dados dos sensores.
        
        r0.client->Read();
        curr_pos = odom_pos = r0.navigator->GetPose();
        vx_robot = r0.navigator->GetXSpeed();
        vy_robot = r0.navigator->GetYSpeed();
        va_robot = r0.navigator->GetYawSpeed();
        // Calculando erro de posição da trajetória planejada com leitura do odometro
//         x_diff = (path_log_pos.px - curr_pos.px);
//         y_diff = (path_log_pos.py - curr_pos.py);
//         angle_diff = normalize_angle(path_log_pos.pa - curr_pos.pa);
        
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
        vx_control = limit_speed(vx_path + p_trans_error, 0.7);
        va_control = va_path + p_rot_error;

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
        odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
        gettimeofday(&t_end, NULL);
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        r0.navigator->AdjustSpeed(vx_control, va_control);
        data_fp << "CONTROLLER" << endl;
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0) * 1e6);
    }
        
    path_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    odom_pos = r0.navigator->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
    odom_fp.close();
    data_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
}

void PlayerTracking::control_02(char *log, char *ip)
{
    int i;
    //double it, states[n], aux1[n], aux2[n], control[2];
    double data[robot_model->n_states];
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
    double kp_trans=1.05*wp, kp_rot=1.05*wp;
    double p_trans_error, p_rot_error;
    Robot r0(ip);
    // Delay
    struct timeval t_begin, t_end;
    double t0, t1, dt;
    // End Delay
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
    for (i=1; i<robot_model->n_states; i++)
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
    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    r0.client->Read();
    old_pos = r0.navigator->GetPose();
    
    for(i=0;i<10;i++)
        r0.client->Read();
    
    while(path_fp.getline(temp, 100))
    {
        for(i=0;i<2;i++)
        {
            gettimeofday(&t_begin, NULL);
            t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
            // Requisita dados dos sensores.
            r0.client->Read();
            curr_pos = odom_pos = r0.navigator->GetPose();
//             vx_robot = r0.navigator->GetXSpeed();
//             vy_robot = r0.navigator->GetYSpeed();
//             va_robot = r0.navigator->GetYawSpeed();
            // Calculando erro de posição da trajetória planejada com leitura do odometro
            x_diff = (old_path_pos.px - curr_pos.px);
            y_diff = (old_path_pos.py - curr_pos.py);
            angle_diff = normalize_angle(old_path_pos.pa - curr_pos.pa);
            // Preenchendo estrutura auxiliar
            aux_dq.px = x_diff;
            aux_dq.py = y_diff;
            aux_dq.pa = angle_diff;
            // Transformando para coordenas referentes ao angulo local
            speed_error = calculate_speed(aux_dq, curr_pos.pa);
            p_trans_error = kp_trans * speed_error.px;
            p_rot_error = kp_rot * speed_error.pa;
            vx_control = limit_speed(vx_path + p_trans_error, 0.7);
            va_control = va_path + p_rot_error;
            // Logando informações
//             data_fp << "x_path: " << path_log_pos.px << " y_path: "
//                     << path_log_pos.py << " angle_path: " << path_log_pos.pa << endl;
//             data_fp << "vx_path: " << vx_path << " va_path: " << va_path << endl;
//             data_fp << "x_odom: " << odom_pos.px << " y_odom: " << odom_pos.py
//                     << " angle_odom: " << odom_pos.pa << endl;
//             data_fp << "vx_robot: " << vx_robot << " vy_robot: " << vy_robot
//                     << " va_robot: " << va_robot << endl;
            data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << angle_diff << endl;
            data_fp << "speed_error_x: " << speed_error.px << " speed_error_a: " << speed_error.pa << endl;
            data_fp << "vx_control: " << vx_control << " va_control: " << va_control << endl;
            odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
            // Fim do Log
            old_pos = curr_pos;
            gettimeofday(&t_end, NULL);
            t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
            dt = t1-t0;
            r0.navigator->AdjustSpeed(vx_control, va_control);
            data_fp << "PROCESSING TIME: " << dt << endl << endl;
            if(dt < INTEGRATION_TIME/2.0)
                usleep((INTEGRATION_TIME/2.0 - dt) * 1e6);
        }
        data[0] = strtod(temp, &ps);
        nxt = ps;
        for (i=1; i<robot_model->n_states; i++)
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
    }
    while(fabs(x_diff) > 0.05 || fabs(angle_diff) > 0.05)
    {
        vx_path = 0.0;
        va_path = 0.0;
       gettimeofday(&t_begin, NULL);
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // Requisita dados dos sensores.
        
        r0.client->Read();
        curr_pos = odom_pos = r0.navigator->GetPose();
        vx_robot = r0.navigator->GetXSpeed();
        vy_robot = r0.navigator->GetYSpeed();
        va_robot = r0.navigator->GetYawSpeed();
        // Calculando erro de posição da trajetória planejada com leitura do odometro
//         x_diff = (path_log_pos.px - curr_pos.px);
//         y_diff = (path_log_pos.py - curr_pos.py);
//         angle_diff = normalize_angle(path_log_pos.pa - curr_pos.pa);
        
        x_diff = (old_path_pos.px - curr_pos.px);
        y_diff = (old_path_pos.py - curr_pos.py);
        angle_diff = normalize_angle(old_path_pos.pa - curr_pos.pa);
        // Preenchendo estrutura auxiliar
        aux_dq.px = x_diff;
        aux_dq.py = y_diff;
        aux_dq.pa = angle_diff;
        // Transformando para coordenas referentes ao angulo local
        speed_error = calculate_speed(aux_dq, curr_pos.pa);
        p_trans_error = kp_trans * speed_error.px;
        p_rot_error = kp_rot * speed_error.pa;
        vx_control = limit_speed(vx_path + p_trans_error, 0.7);
        va_control = va_path + p_rot_error;

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
        odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
        gettimeofday(&t_end, NULL);
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        r0.navigator->AdjustSpeed(vx_control, va_control);
        data_fp << "CONTROLLER" << endl;
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0) * 1e6);
    }
        
    path_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    odom_pos = r0.navigator->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
    odom_fp.close();
    data_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
}

void PlayerTracking::ParseLog(const char *log_str, Pioneer3ATState *state)
{
    double data[robot_model->n_states];
    char  *nxt, *ps;
    data[0] = strtod(log_str, &ps);
    nxt = ps;
    for (int i=1; i<robot_model->n_states; i++)
    {
        data[i] = strtod(nxt, &ps);
        nxt = ps;
    }
    state->x = data[STATE_X];
    state->y = -data[STATE_Y];
    state->psi = -data[STATE_THETA];
    state->v = data[STATE_V];
    state->w = -data[STATE_W];
}

void PlayerTracking::NoControl(const char *log, const char *ip)
{
    struct timeval t_begin, t_end;
    double t0, t1;
    char temp[100];
    ifstream path_fp(log);
    player_pose2d_t path_log_pos, odom_pos;
    ofstream odom_fp(ODOM_LOG_FILE);
    ofstream data_fp(DATA_LOG_FILE);
    Robot r0(ip);

    Pioneer3ATState curr_state, ref_state;

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

    path_fp.getline(temp,100);
    ParseLog(temp, &curr_state);
    ref_state = curr_state;
    // Preenchendo estrutura auxiliar para ajustar odometro para posição inicial
    path_log_pos.px = curr_state.x;
    path_log_pos.py = curr_state.y;
    path_log_pos.pa = curr_state.psi;

    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    while(path_fp.getline(temp, 100))
    {
        gettimeofday(&t_begin, NULL);
        r0.client->Read();
        // Requisita dados dos sensores.
        curr_state = r0.navigator->GetRobotState();
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
        ParseLog(temp, &ref_state);
        r0.navigator->AdjustSpeed(ref_state.v, ref_state.w);
        gettimeofday(&t_end, NULL);        
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        if((t1-t0) < INTEGRATION_TIME)
        {
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0)*1e6);
//             cout << (t1-t0) << " seconds." << endl;
        }
    }
    path_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    data_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    odom_pos = r0.navigator->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
}

void PlayerTracking::ControlByFierro(const char *log, const char *ip)
{
    struct timeval t_begin, t_end;
    double t0, t1;
    char temp[100];
    ifstream path_fp(log);
    player_pose2d_t path_log_pos, odom_pos;
    ofstream odom_fp(ODOM_LOG_FILE);
    ofstream data_fp(DATA_LOG_FILE);
    double u[2], computed_torques[2], vel_tracking[2];
    double curr_st[robot_model->n_states], ref_st[robot_model->n_states];
    Robot r0(ip);

    Pioneer3ATState curr_state, ref_state;

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

    path_fp.getline(temp,100);
    ParseLog(temp, &curr_state);
    ref_state = curr_state;
    ref_st[0] = ref_state.x;
    ref_st[1] = ref_state.y;
    ref_st[2] = ref_state.psi;
    ref_st[3] = ref_state.v;
    ref_st[4] = ref_state.w;
    // Preenchendo estrutura auxiliar para ajustar odometro para posição inicial
    path_log_pos.px = curr_state.x;
    path_log_pos.py = curr_state.y;
    path_log_pos.pa = curr_state.psi;

    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    while(path_fp.getline(temp, 100))
    {
        gettimeofday(&t_begin, NULL);
        r0.client->Read();
        // Requisita dados dos sensores.
        curr_state = r0.navigator->GetRobotState();
        curr_st[0] = curr_state.x;
        curr_st[1] = curr_state.y;
        curr_st[2] = curr_state.psi;
        curr_st[3] = curr_state.v;
        curr_st[4] = curr_state.w;
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
        ((SkidSteerControlBased*)robot_model)->trajectory_control->run(curr_st, ref_st, u);
        ((SkidSteerControlBased*)robot_model)->EstimateTorque(curr_st, u, computed_torques);
        ((SkidSteerControlBased*)robot_model)->EstimateVelocitiesFromTorque(curr_st, computed_torques, vel_tracking);
        // Atualizando estado de referência a partir da leitura do log de planejamento
        ParseLog(temp, &ref_state);
        ref_st[0] = ref_state.x;
        ref_st[1] = ref_state.y;
        ref_st[2] = ref_state.psi;
        ref_st[3] = ref_state.v;
        ref_st[4] = ref_state.w;
        r0.navigator->AdjustSpeed(vel_tracking[0], vel_tracking[1]);
        gettimeofday(&t_end, NULL);        
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        if((t1-t0) < INTEGRATION_TIME)
        {
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0)*1e6);
//             cout << (t1-t0) << " seconds." << endl;
        }
    }
    path_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    r0.client->Read();
    data_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
    odom_pos = r0.navigator->GetPose();
    odom_fp << odom_pos.px << " " << -odom_pos.py << " " << -odom_pos.pa << endl;
}
