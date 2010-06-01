#ifndef _CONSTANTS_RRT_MOTION_PLANNER_
#define _CONSTANTS_RRT_MOTION_PLANNER_

#define GOAL_TRANSLATIONAL_TOLERANCE 0.2
//5 graus
#define ROTATIONAL_TOLERANCE 0.087
//10 graus
#define MAX_ROTATIONAL_TOLERANCE 2 * ROTATIONAL_TOLERANCE
//mudanca maxima de angulo de estercamento em rad (100 graus/(s*s))
#define MAX_STEER_ACCEL 1.7453292519943295
//angulo maximo de estercamento em rad (30 graus)
#define MAX_STEERING 0.523598
//mudanca maxima de velocidade
#define MAX_LIN_ACCEL 0.3
//velocidade maxima
#define MAX_SPEED 1.0

#define DELTA_T 0.04
#define INTEGRATION_TIME 0.2

#define GOAL_BIAS 0.15

#define N_STATES 3
#define N_CONTROLS 11

#define SCALE_FACTOR 50.0

#define COLLISION_TOLERANCE 0.15
#define COLLIDED 1

#define GRAVITY 9.81

enum
{
    STATE_X=0, STATE_Y, STATE_THETA, STATE_V, STATE_W
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

#endif