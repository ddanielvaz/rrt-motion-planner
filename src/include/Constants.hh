#ifndef __RRT_CONSTANTS_HH__
#define __RRT_CONSTANTS_HH__

#define GOAL_TRANSLATIONAL_TOLERANCE 0.2
//5 graus
#define ROTATIONAL_TOLERANCE 0.087
//10 graus
#define MAX_ROTATIONAL_TOLERANCE 2 * ROTATIONAL_TOLERANCE

#define MAX_LIN_SPEED_TOLERANCE 0.05
#define MAX_ROT_SPEED_TOLERANCE 0.05

//angulo maximo de estercamento em rad (30 graus)
#define MAX_STEERING_ANGLE 0.523598
//velocidade maxima - datasheet 0.75 m/s, 100 graus/s ou 1.745 rad/s
//adotando velocidade menor para translação de 0.5 m/s e 1.2 rad/s
#define MAX_LIN_SPEED 0.5
#define AVG_LIN_SPEED 0.3
#define STD_DEVIATION_LIN_SPEED 0.2
#define MAX_ROT_SPEED 0.8

#define DELTA_T 0.2
#define INTEGRATION_TIME 0.2

#define GOAL_BIAS 0.15
#define AVG_BIAS 0.50

#define P3AT_N_STATES 5

#define COLLISION_TOLERANCE 0.15
#define COLLIDED 1

#define GRAVITY 9.81

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

enum
{
    u1=0, u2
};

#endif
