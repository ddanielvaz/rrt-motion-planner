#ifndef _CONSTANTS_RRT_MOTION_PLANNER_
#define _CONSTANTS_RRT_MOTION_PLANNER_

#define GOAL_TRANSLATIONAL_TOLERANCE 0.2
//5 graus
#define ROTATIONAL_TOLERANCE 0.087
//10 graus
#define MAX_ROTATIONAL_TOLERANCE 2 * ROTATIONAL_TOLERANCE
//angulo maximo de estercamento em rad (30 graus)
#define MAX_STEERING_ANGLE 0.523598
//velocidade maxima - datasheet 0.75 m/s, 100 graus/s ou 1.745 rad/s
//adotando velocidade menor para translação de 0.5 m/s e 1.2 rad/s
#define MAX_LIN_SPEED 0.5
#define MAX_ROT_SPEED 1.2

#define DELTA_T 0.04
#define INTEGRATION_TIME 0.2

#define GOAL_BIAS 0.15

#define N_STATES 3
#define N_CONTROLS 11

#define COLLISION_TOLERANCE 0.15
#define COLLIDED 1

#define GRAVITY 9.81

#define TORQUE_LOGFILE "p3at.torques"
#define ACCEL_LOGFILE "p3at.accel"
#define CARLIKE_LOGFILE "carlike.accel"

enum
{
    STATE_X=0, STATE_Y, STATE_THETA, STATE_V, STATE_W
};

enum{
    STATE_STEERING_ANGLE=4
};

enum
{
    CONTROL_VELOCITY=0, CONTROL_STEERING_ANGLE
};

enum
{
    VX_SPEED=0, ANGULAR_SPEED
};

enum
{
    TORQUE_R=0, TORQUE_L
};

enum
{
    LINEAR_ACCEL=0, ANGULAR_ACCEL
};

enum
{
    DV_DESIRED=0, DW_DESIRED
};

typedef struct
{
    double ctrl[2];
}control_input;

#endif
