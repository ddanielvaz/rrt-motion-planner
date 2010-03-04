#ifndef _Rapidly_Random_Tree_H_
#define _Rapidly_Random_Tree_H_

#include <fstream>
#include <iostream>
#include <list>
#include <stdlib.h>

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

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
        RRT(double *, double *, int, ModelCar*, World*, char*);
        void build(void);
        int extend(double*);
        double* select_best_input(double*);
        Node select_nearest_node(double*);
        void get_control_inputs(double u[][2], double*);
        bool check_no_collision_path(const double*, const double*, double*);
        int check_duplicate_node(double *);
        void print_path(const double*, const double*);
        void close_logfile(void);
        ~RRT();
    private:
        Node initial_node, goal_node;
        double goal_state[3], initial_state[3];
        int max_nodes;
        ModelCar *veh;
        World *env;
        ofstream fp;
        Graph g;
        ArcMapInt *cost;
        NodeMapState *states;
};

RRT::RRT(double *init, double *goal, int n, ModelCar *car, World *w, char *fname)
{
    cout << "Criando instancia da classe RRT" << endl;
    states = new NodeMapState(g);
    cost = new ArcMapInt(g);
    initial_node = g.addNode();
    (*states)[initial_node] = init;
    memcpy(goal_state, goal, sizeof(double) * 3);
    memcpy(initial_state, init, sizeof(double) * 3);
    max_nodes = n;
    veh = car;
    env = w;
    fp.open(fname);
    initiate_rand_number_generator();
}

RRT::~RRT()
{
    cout << "Destruindo instancia da classe RRT" << endl;
}

void RRT::build(void)
{
    int finished=0, i=0;
    double rand_st[3];
    while(i < max_nodes)
    {
        //Se amostra maior que GOAL_BIAS, ponto aleatorio eh escolhido para expandir
        //arvore. Caso contrario eh escolhido o destino para a expansao.
        if(uni() > GOAL_BIAS+2)
            biased_sampling(env->dim, rand_st);
        else
            memcpy(rand_st, goal_state, sizeof(double) * 3);
        finished = extend(rand_st);
        if(finished == 2)
        {
            cout << "[" << i << "] Objetivo alcancado." << endl;
            //print_nodes(nodes);
            break;
        }
        else if(finished == 1)
            i++;
    }
}


int RRT::extend(double *rand)
{
    Node near = select_nearest_node(rand), added;
    double expanded[3], *pt=NULL, *choosed=NULL, *best_control=NULL, temp[3];
    bool not_collided;
    double u[21][2]={{0.50, 0.00}, {0.50, 0.02}, {0.50, 0.07}, {0.50, 0.12}, {0.50, 0.17},
                     {0.50, 0.23}, {0.50, 0.28}, {0.50, 0.33}, {0.50, 0.38},
                     {0.50, 0.44}, {0.50, 0.49}, {-0.50, 0.00}, {-0.50, 0.07},
                     {-0.50, 0.12}, {-0.50, 0.17}, {-0.50, 0.23}, {-0.50, 0.28},
                     {-0.50, 0.33}, {-0.50, 0.38}, {-0.50, 0.44}, {-0.50, 0.49}
                    };
    double d=1e7, aux;
    int duplicated_node_id;
    for(int i=0; i<21; i++)
    {
        not_collided = check_no_collision_path((*states)[near], u[i], expanded);
        if (not_collided)
        {
            aux = metric(expanded, rand);
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
        duplicated_node_id = check_duplicate_node(temp);
        //cout << "ID: " << duplicated_node_id << " ";
        if (duplicated_node_id >= 0)
        {
            //cout << "Duplicated NODE detect ID: " << duplicated_node_id << endl;
            Node n = g.nodeFromId(duplicated_node_id);
            g.addArc(near, n);
            return -1;
        }
        print_path((*states)[near], best_control);
        choosed = (double*) malloc(sizeof(double) * 3);
        memcpy(choosed, temp, sizeof(double) * 3);
        added = g.addNode();
        (*states)[added] = choosed;
        g.addArc(near, added);
        if(goal_state_reached((*states)[added], goal_state))
            return 2;
        else
            return 1;
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
        if (aux < min_distance)
        {
            min_distance = aux;
            nearest = n;
        }
    }
    return nearest;
}

bool RRT::check_no_collision_path(const double *near_node, const double *u, double *new_state)
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
    for (it=0.0; it<=INTEGRATION_TIME; it+=DELTA_T)
    {
        veh->EstimateNewState(DELTA_T, aux, u, temp);
        trans[STATE_X] = temp[STATE_X] - x0;
        trans[STATE_Y] = temp[STATE_Y] - y0;
        dtheta = normalize_angle(temp[STATE_THETA] - theta0);
        rot[1][1] = rot[0][0] = cos(dtheta);
        rot[1][0] = sin(dtheta);
        rot[0][1] = -rot[1][0];
        in_collision = env->is_vehicle_in_collision(trans, rot);
        if (in_collision)
        {
            return false;
        }
        memcpy(aux, temp, sizeof(double) * 3);
    }
    memcpy(new_state, aux, sizeof(double) * 3);
    return true;
}


int RRT::check_duplicate_node(double *adding)
{
    double aux;
    for(Graph::NodeIt n(g); n != INVALID; ++n)
    {
        //cout << 
        aux = metric((*states)[n], adding);
        if (aux < 1e-2)
        {
            return g.id(n);
        }
    }
    return -1;
}

void RRT::print_path(const double *near_node, const double *best_control)
{
    fp << near_node[0] << " " << near_node[1] << " " << near_node[2] << " "
    << best_control[0] << " " << best_control[1] << " " << INTEGRATION_TIME << endl;
    /*double aux[3], temp[3], it;
    memcpy(aux, near_node, sizeof(double) * 3);
    for (it=0.0; it<=INTEGRATION_TIME; it+=DELTA_T)
    {
        cout << aux[0] << " " << aux[1] << endl;
        veh->EstimateNewState(DELTA_T, aux, best_control, temp);
        memcpy(aux, temp, sizeof(double) * 3);
    }*/
}

void RRT::close_logfile(void)
{
    fp.close();
}

#endif
