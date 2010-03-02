#ifndef _Rapidly_Random_Tree_H_
#define _Rapidly_Random_Tree_H_

#include <iostream>
#include <list>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>
#include <stdlib.h>

#include "simple_car.h"
#include "utils.h"
#include "world.h"

using namespace std;
using namespace lemon;

typedef ListDigraph Graph;
typedef Graph::Node Node;
typedef Graph::ArcMap<int> ArcMapInt;
typedef Graph::NodeMap<double*> NodeMapState;
class RRT
{
    public:
        RRT(double *, double *, int, ModelCar*, World*);
        void build(void);
        int extend(double*);
        double* select_best_input(double*);
        Node select_nearest_node(double*);
        void get_control_inputs(double u[][2], double*);
        bool generate_path(const double*, const double*, const double,
                           ModelCar*, double*);
        ~RRT();
    private:
        Node initial_node, goal_node;
        double goal_state[3], initial_state[3];
        int max_nodes;
        ModelCar *veh;
        World *env;
        Graph g;
        ArcMapInt *cost;
        NodeMapState *states;
};

RRT::RRT(double *init, double *goal, int n, ModelCar *car, World *w)
{
    states = new NodeMapState(g);
    cost = new ArcMapInt(g);
    cout << "Criando instancia da classe RRT" << endl;
    //states = new Graph::ArcMap<double*>(g);
    initial_node = g.addNode();
    //goal_state = g.addNode();
    (*states)[initial_node] = init;
    //(*states)[goal_state] = goal;
    memcpy(goal_state, goal, sizeof(double) * 3);
    memcpy(initial_state, init, sizeof(double) * 3);
    max_nodes = n;
    veh = car;
    env = w;
    initiate_rand_number_generator();
}

RRT::~RRT()
{
    cout << "Destruindo instancia da classe RRT" << endl;
    //delete initial_state;
    //delete goal_state;
}

void RRT::build(void)
{
    //cout << uni() << endl;
    int finished=0, i=0;
    double rand_st[3];
    while(i<max_nodes)
    {
        if(uni() > 0.05)
        {
            biased_sampling(env->dim, rand_st);
            //cout << "RAND: " << rand_st[0] << " " << rand_st[1] << " " << rand_st[2] << endl;
        }
        else
        {
            //cout << "[Privilegiando GOAL]" << endl;
            memcpy(rand_st, goal_state, sizeof(double) * 3);
        }
        finished = extend(rand_st);
        if(finished == 1)
        {
            cout << "[" << i << "] Objetivo alcancado." << endl;
            //print_nodes(nodes);
            break;
        }
        else if(finished == 2)
            i++;
    }
}

int RRT::extend(double *rand)
{
    Node near = select_nearest_node(rand), added;
    double expanded[3], *pt=NULL, *choosed=NULL, *best_control=NULL, temp[3];
    bool new_node=false;
    double u[21][2]={{0.50, 0.00}, {0.50, 0.02}, {0.50, 0.07}, {0.50, 0.12}, {0.50, 0.17},
                     {0.50, 0.23}, {0.50, 0.28}, {0.50, 0.33}, {0.50, 0.38},
                     {0.50, 0.44}, {0.50, 0.49}, {-0.50, 0.00}, {-0.50, 0.07},
                     {-0.50, 0.12}, {-0.50, 0.17}, {-0.50, 0.23}, {-0.50, 0.28},
                     {-0.50, 0.33}, {-0.50, 0.38}, {-0.50, 0.44}, {-0.50, 0.49}
                    };
    double d=1e7, aux;
    //get_control_inputs(u, near);
    //cout << "NEAR NODE:" << (*states)[near][0] << " " << (*states)[near][1] << " "
    //<< (*states)[near][2] << endl;
    for(int i=0; i<20; i++)
    {
        new_node = generate_path((*states)[near], u[i], TIME_STEP, veh, expanded);
        if (new_node)
        {
            aux = metric(expanded, rand);
            //cout << "aux: " << aux << " State: " << expanded[0] << " " << expanded[1] << " " << expanded[2] << endl;
            if(aux < d)
            {
                d = aux;
                memcpy(temp, expanded, sizeof(double) * 3);
                pt = temp;
                best_control = u[i];
            }
        }
    }
    
    if (pt)
    {
        //cout << "BEST CONTROL U: " << best_control[0] << " " << best_control[1] << endl;
        choosed = (double*) malloc(sizeof(double) * 3);
        memcpy(choosed, temp, sizeof(double) * 3);
        added = g.addNode();
        (*states)[added] = choosed;
        //cout << "no adicionado: " << expanded[0] << " " << expanded[1] << " " << expanded[2] << endl;
        cout << expanded[0] << " " << expanded[1] << endl;
        if(goal_state_reached((*states)[added], goal_state))
        {
            cout << "GOAL STATE REACHED - FINISH" << endl;
            return 1;
        }
        return 2;
    }
    return -1;
}

Node RRT::select_nearest_node(double *rand)
{
    double min_distance = 1e7, aux;
    Node nearest;
    for(Graph::NodeIt n(g); n != INVALID; ++n)
    {
        aux = metric((*states)[n], rand);
        //cout << aux << " " << (*states)[n][0] << " " << (*states)[n][1] << " " << (*states)[n][2] << endl;
        if (aux < min_distance)
        {
            min_distance = aux;
            nearest = n;
        }
    }
    return nearest;
}

bool RRT::generate_path(const double *near_node, const double *u, const double t,
                         ModelCar *car, double *new_state)
{
    double it, aux[3], temp[3], dtheta, x0, y0, theta0;
    bool in_collision;
    PQP_REAL trans[3], rot[3][3];
    trans[2] = 0.0;
    rot[2][0] = rot[2][1] = 0.0;
    rot[0][2] = rot[1][2] = 0.0;
    rot[2][2] = 1.0;
    memcpy(aux, near_node, sizeof(double) * 3);
    x0 = initial_state[STATE_X];
    y0 = initial_state[STATE_Y];
    theta0 = initial_state[STATE_THETA];
    for (it=0.0; it<=t; it+=DELTA_T)
    {
        car->EstimateNewState(DELTA_T, aux, u, temp);
        trans[STATE_X] = temp[STATE_X] - x0;
        trans[STATE_Y] = temp[STATE_Y] - y0;
        //cout << "trans[x]= " << trans[STATE_X] << ", trans[y]= " << trans[STATE_Y] << endl;
        dtheta = temp[STATE_THETA] - theta0;
        rot[1][1] = rot[0][0] = cos(dtheta);
        rot[1][0] = sin(dtheta);
        rot[0][1] = -rot[1][0];
        in_collision = env->is_vehicle_in_collision(trans, rot);
        if (in_collision)
        {
            //cout << "collision detected" << endl;
            return false;
        }
        memcpy(aux, temp, sizeof(double) * 3);
    }
    memcpy(new_state, aux, sizeof(double) * 3);
    return true;
}

#endif
