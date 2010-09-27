// #include "position2d_proxy.h"

class pioneer_3at_state
{
    public:
        pioneer_3at_state(const double *state);
        double x,y,psi,v,w;
};

pioneer_3at_state::pioneer_3at_state(const double *state)
{
//     cout << "Initializing pioneer 3AT states - (x,y,psi,v,w)" << endl;
    x = state[STATE_X];
    y = state[STATE_Y];
    psi = state[STATE_THETA];
    v = state[STATE_V];
    w = state[STATE_W];
}

class TrackingControlPioneer3AT
{
    public:
        void run(const double *, const double *, const double *, double *);
};

/**
@brief: A partir do estado atual, estado futuro (estado de referência) e entrada de controle, através de uma lei de seguimento de trajetória 
*/
void TrackingControlPioneer3AT::run(const double *curr_st, const double *ref_st,
                          const double *ctl, double *tracked_st)
{
    pioneer_3at_state curr_state(curr_st);
    pioneer_3at_state ref_state(ref_st);
    static int i=0;
    if(i<3){

    cout << "curr_state.x: " << curr_state.x << " curr_state.y: " << curr_state.y
         << " curr_state.psi: " << curr_state.psi << " curr_state.v: " << curr_state.v
         << " curr_state.w: " << curr_state.w << endl;

    cout << "ref_state.x: " << ref_state.x << " ref_state.y: " << ref_state.y
         << " ref_state.psi: " << ref_state.psi << " ref_state.v: " << ref_state.v
         << " ref_state.w: " << ref_state.w << endl;

    cout << "linear accel: " << ctl[LINEAR_ACCEL] << " angular accel: "
         << ctl[ANGULAR_ACCEL] << endl;
         i++;
    }
    memcpy(tracked_st, ref_st, sizeof(double) * 5);
}
//     Calculando erro de posição da trajetória planejada com leitura do odometro
//     x_diff = (old_path_pos.px - curr_pos.px);
//     y_diff = (old_path_pos.py - curr_pos.py);
//     angle_diff = normalize_angle(old_path_pos.pa - curr_pos.pa);
//     Preenchendo estrutura auxiliar
//     aux_dq.px = x_diff;
//     aux_dq.py = y_diff;
//     aux_dq.pa = angle_diff;
//     Transformando para coordenas referentes ao angulo local
//     speed_error = calculate_speed(aux_dq, curr_pos.pa);
//     p_trans_error = kp_trans * speed_error.px;
//     p_rot_error = kp_rot * speed_error.pa;
//     vx_control = limit_speed(vx_path + p_trans_error, 0.7);
//     va_control = va_path + p_rot_error;
// 
//     Logando informações
//     data_fp << "x_path: " << path_log_pos.px << " y_path: "
//             << path_log_pos.py << " angle_path: " << path_log_pos.pa << endl;
//     data_fp << "vx_path: " << vx_path << " va_path: " << va_path << endl;
//     data_fp << "x_odom: " << odom_pos.px << " y_odom: " << odom_pos.py
//             << " angle_odom: " << odom_pos.pa << endl;
//     data_fp << "vx_robot: " << vx_robot << " vy_robot: " << vy_robot
//             << " va_robot: " << va_robot << endl;
//     data_fp << "x_diff: " << x_diff << " y_diff: " << y_diff << " angle_diff:" << angle_diff << endl;
//     data_fp << "speed_error_x: " << speed_error.px << " speed_error_a: " << speed_error.pa << endl;
//     data_fp << "vx_control: " << vx_control << " va_control: " << va_control
//             << endl << endl;
