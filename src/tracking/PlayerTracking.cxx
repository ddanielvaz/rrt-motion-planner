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


void PlayerTracking::Kanayama(const char *log, const char *ip)
{
    struct timeval t_begin, t_end;
    double t0, t1;
    char temp[100];
    ifstream path_fp(log);
    player_pose2d_t path_log_pos;
    ofstream odom_fp(ODOM_LOG_FILE);
    ofstream data_fp(DATA_LOG_FILE);
    Robot r0(ip);
    Pioneer3ATState curr_state, ref_state;
    double x_diff, y_diff, psi_diff;
    double x_e, y_e, psi_e;
    double vx_control, va_control;
    double Kx, Ky, Ktheta;
    Kx = 0.1;
    Ky = 0.1;
    Ktheta = 0.1;
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
    ParseLog(temp, &ref_state);
    // Preenchendo estrutura auxiliar para ajustar odometro para posição inicial
    path_log_pos.px = ref_state.x;
    path_log_pos.py = ref_state.y;
    path_log_pos.pa = ref_state.psi;

    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    while(path_fp.getline(temp, 100))
    {
        gettimeofday(&t_begin, NULL);
        r0.client->Read();
        // Requisita dados dos sensores.
        curr_state = r0.navigator->GetRobotState();
        x_diff = ref_state.x - curr_state.x;
        y_diff = ref_state.y - curr_state.y;
        psi_diff = normalize_angle(ref_state.psi - curr_state.psi);
        x_e = cos(curr_state.psi) * x_diff + sin(curr_state.psi) * y_diff;
        y_e = cos(curr_state.psi) * y_diff - sin(curr_state.psi) * x_diff;
        psi_e = psi_diff;
        vx_control = ref_state.v * cos(psi_e) + Kx * x_e;
        va_control = ref_state.w + ref_state.v*(Ky*y_e + Ktheta * sin(psi_e));
        // Logando informações
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
        data_fp << "vx_path: " << ref_state.v << " va_path: " << ref_state.w << endl;
        data_fp << "vx_robot: " << curr_state.v << " va_robot: " << curr_state.w << endl;
        data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << psi_diff << endl;
        data_fp << "vx_control: " << vx_control << " va_control: " << va_control << endl << endl;
        // Fim do Log
        ParseLog(temp, &ref_state);
        gettimeofday(&t_end, NULL);        
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0)*1e6);
        r0.navigator->AdjustSpeed(vx_control, va_control);
    }
    path_fp.close();
    odom_fp.close();
    data_fp.close();
    r0.navigator->AdjustSpeed(0.0, 0.0);
}

void PlayerTracking::ProportionalController(const char *log, const char *ip)
{
    char temp[100];
    ifstream path_fp(log);
    player_pose2d_t speed_error, aux;
    ofstream odom_fp("odom.log");
    ofstream data_fp("data.log");
    double x_diff, y_diff, psi_diff;
    double vx_control, va_control;
    double kp_trans=0.1, kp_rot=0.01;
    double p_trans_error, p_rot_error;
    struct timeval t_begin, t_end;
    double t0, t1;
    Pioneer3ATState curr_state, ref_state;
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
    ParseLog(temp, &ref_state);
    aux.px = ref_state.x;
    aux.py = ref_state.y;
    aux.pa = ref_state.psi;
    r0.navigator->SetOdomPos(aux);
    r0.navigator->SetMotorStatus(true);
    r0.client->Read();
    // Fazendo várias leituras para garantir descartar dados antigos.
    for(int i=0;i<10;i++)
        r0.client->Read();
    while(path_fp.getline(temp, 100))
    {
        gettimeofday(&t_begin, NULL);
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // Requisita dados dos sensores.
        r0.client->Read();
        curr_state = r0.navigator->GetRobotState();
        // Calculando erro de posição da trajetória planejada com leitura do odometro
        x_diff = (ref_state.x - curr_state.x);
        y_diff = (ref_state.y - curr_state.y);
        psi_diff = normalize_angle(ref_state.psi - curr_state.psi);
        // Preenchendo estrutura auxiliar
        aux.px = x_diff;
        aux.py = y_diff;
        aux.pa = psi_diff;
        // Transformando para coordenas referentes ao angulo local
        speed_error = calculate_speed(aux, curr_state.psi);
        p_trans_error = kp_trans * speed_error.px;
        p_rot_error = kp_rot * speed_error.pa;
        vx_control = limit_speed(ref_state.v + p_trans_error, 0.7);
        va_control = ref_state.w + p_rot_error;

        // Logando informações
        data_fp << "vx_path: " << ref_state.v << " va_path: " << ref_state.w << endl;
        data_fp << "vx_robot: " << curr_state.v << " va_robot: " << curr_state.w << endl;
        data_fp << "v_diff: " << curr_state.v - ref_state.v << " w_diff_robot: " << curr_state.w - ref_state.w << endl;
        data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << psi_diff << endl;
        data_fp << "vx_control: " << vx_control << " va_control: " << va_control << endl << endl;
        // Fim do Log
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
        // Atualizando estado de referência a partir da leitura do log de planejamento
        ParseLog(temp, &ref_state);
        // Tempo de execução
        gettimeofday(&t_end, NULL);
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        data_fp << "PROCESSING TIME: " << (t1-t0) << endl;
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0) * 1e6);
        r0.navigator->AdjustSpeed(vx_control, va_control);
    }
    while(fabs(x_diff) > 0.05 || fabs(psi_diff) > 0.05)
    {
        ref_state.v = 0.0;
        ref_state.w = 0.0;
        gettimeofday(&t_begin, NULL);
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        // Requisita dados dos sensores.
        
        r0.client->Read();
        curr_state = r0.navigator->GetRobotState();
        
        x_diff = (ref_state.x - curr_state.x);
        y_diff = (ref_state.y - curr_state.y);
        psi_diff = normalize_angle(ref_state.psi - curr_state.psi);
        // Preenchendo estrutura auxiliar
        aux.px = x_diff;
        aux.py = y_diff;
        aux.pa = psi_diff;
        // Transformando para coordenas referentes ao angulo local
        speed_error = calculate_speed(aux, curr_state.psi);
        p_trans_error = kp_trans * speed_error.px;
        p_rot_error = kp_rot * speed_error.pa;
        vx_control = limit_speed(p_trans_error, 0.7);
        va_control = p_rot_error;

        // Logando informações
        data_fp << "vx_path: " << ref_state.v << " va_path: " << ref_state.w << endl;
        data_fp << "vx_robot: " << curr_state.v << " va_robot: " << curr_state.w << endl;
        data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << psi_diff << endl;
        data_fp << "vx_control: " << vx_control << " va_control: " << va_control << endl << endl;
        // Fim do Log
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
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
    odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
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
    player_pose2d_t path_log_pos;
    ofstream odom_fp(ODOM_LOG_FILE);
    ofstream data_fp(DATA_LOG_FILE);
    Robot r0(ip);
    Pioneer3ATState curr_state, ref_state, old_ref_state;
    double x_diff, y_diff, psi_diff;

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
        cout << e << endl;
        return;
    }
    r0.client->Read();

    path_fp.getline(temp,100);
    ParseLog(temp, &ref_state);
    // Preenchendo estrutura auxiliar para ajustar odometro para posição inicial
    path_log_pos.px = ref_state.x;
    path_log_pos.py = ref_state.y;
    path_log_pos.pa = ref_state.psi;
    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    while(true)
    {
        gettimeofday(&t_begin, NULL);
        old_ref_state = ref_state;
        if(!path_fp.getline(temp, 100))
            break;
        ParseLog(temp, &ref_state);
        r0.client->Read();
        // Requisita dados dos sensores.
        curr_state = r0.navigator->GetRobotState();
        x_diff = old_ref_state.x - curr_state.x;
        y_diff = old_ref_state.y - curr_state.y;
        psi_diff = normalize_angle(old_ref_state.psi - curr_state.psi);
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
        // Logando informações
        data_fp << "vx_path: " << ref_state.v << " va_path: " << ref_state.w << endl;
        data_fp << "vx_robot: " << curr_state.v << " va_robot: " << curr_state.w << endl;
        data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << psi_diff << endl << endl;
        // Fim do Log
        gettimeofday(&t_end, NULL);        
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        r0.navigator->AdjustSpeed(ref_state.v, ref_state.w);
        if((t1-t0) < INTEGRATION_TIME)
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0)*1e6);
    }
    path_fp.close();
    data_fp.close();
    odom_fp.close();
    r0.client->Read();
    r0.navigator->AdjustSpeed(0.0, 0.0);
}

void PlayerTracking::ControlByFierro(const char *log, const char *ip)
{
    struct timeval t_begin, t_end;
    double t0, t1;
    char temp[100];
    ifstream path_fp(log);
    player_pose2d_t path_log_pos;
    ofstream odom_fp(ODOM_LOG_FILE);
    ofstream data_fp(DATA_LOG_FILE);
    double u[2], computed_torques[2], vel_tracking[2];
    double curr_st[robot_model->n_states], ref_st[robot_model->n_states];
    Robot r0(ip);
    Pioneer3ATState curr_state, ref_state;
    double x_diff, y_diff, psi_diff;
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
    ParseLog(temp, &ref_state);
    ref_st[0] = ref_state.x;
    ref_st[1] = ref_state.y;
    ref_st[2] = ref_state.psi;
    ref_st[3] = ref_state.v;
    ref_st[4] = ref_state.w;
    // Preenchendo estrutura auxiliar para ajustar odometro para posição inicial
    path_log_pos.px = ref_state.x;
    path_log_pos.py = ref_state.y;
    path_log_pos.pa = ref_state.psi;
    // Retornando ao início do Arquivo
    path_fp.seekg(0, ios::beg);
    r0.navigator->SetOdomPos(path_log_pos);
    r0.navigator->SetMotorStatus(true);
    while(true)
    {
        gettimeofday(&t_begin, NULL);
        if(!path_fp.getline(temp, 100))
            break;
        // Atualizando estado de referência a partir da leitura do log de planejamento
        ParseLog(temp, &ref_state);
        ref_st[0] = ref_state.x;
        ref_st[1] = ref_state.y;
        ref_st[2] = ref_state.psi;
        ref_st[3] = ref_state.v;
        ref_st[4] = ref_state.w;
        r0.client->Read();
        // Requisita dados dos sensores.
        curr_state = r0.navigator->GetRobotState();
        curr_st[0] = curr_state.x;
        curr_st[1] = curr_state.y;
        curr_st[2] = curr_state.psi;
        curr_st[3] = curr_state.v;
        curr_st[4] = curr_state.w;
        x_diff = ref_state.x - curr_state.x;
        y_diff = ref_state.y - curr_state.y;
        psi_diff = normalize_angle(ref_state.psi - curr_state.psi);
        // Log de dados
        odom_fp << curr_state.x << " " << -curr_state.y << " " << -curr_state.psi << endl;
        data_fp << "v_diff: " << ref_state.v - curr_state.v << " w_diff: " << ref_state.w - curr_state.w << endl;
        data_fp << "vx_path: " << ref_state.v << " va_path: " << ref_state.w << endl;
        data_fp << "vx_robot: " << curr_state.v << " va_robot: " << curr_state.w << endl;
        data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << psi_diff << endl;
        if(ref_state.v<0)
            cout << "VEL NEG"<< endl;
            cout << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << psi_diff << endl;
        ((SkidSteerControlBased*)robot_model)->trajectory_control->run(curr_st, ref_st, u);
        ((SkidSteerControlBased*)robot_model)->EstimateTorque(curr_st, u, computed_torques);
        ((SkidSteerControlBased*)robot_model)->EstimateVelocitiesFromTorque(curr_st, computed_torques, vel_tracking);
        data_fp << "v_track: " << vel_tracking[0] << " w_track: " << vel_tracking[1] << endl << endl;
        gettimeofday(&t_end, NULL);        
        t0 = t_begin.tv_sec + t_begin.tv_usec * 1e-6;
        t1 = t_end.tv_sec + t_end.tv_usec * 1e-6;
        r0.navigator->AdjustSpeed(vel_tracking[0], vel_tracking[1]);
        if((t1-t0) < INTEGRATION_TIME)
        {
            usleep(INTEGRATION_TIME * 1e6 - (t1-t0)*1e6);
        }
    }
    odom_fp.close();
    path_fp.close();
    data_fp.close();
    r0.client->Read();
    r0.navigator->AdjustSpeed(0.0, 0.0);
}
