#ifndef _Rapidly_Random_Tree_H_
#define _Rapidly_Random_Tree_H_

#include <fstream>
#include <iostream>
#include <stdlib.h>

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>
#include <lemon/random.h>

#include "robots.h"
#include "utils.h"
#include "world.h"

using namespace std;
using namespace lemon;

typedef ListDigraph Graph;
typedef Graph::Node Node;
typedef Graph::ArcMap<double> ArcMapWeight;
typedef Graph::NodeMap<double*> NodeMapState;
typedef Graph::ArcMap<double*> ArcMapControl;
typedef Path<Graph> MyPath;

class RRT
{
    public:
        RRT(double *, double *, int, RobotModel*, World*, char*, char*);
        void build(void);
        int extend(double*);
        double* select_best_input(double*);
        Node select_nearest_node(double*);
        void get_control_inputs(double u[][2], double*);
        bool check_no_collision_path(const double*, const double*, double*);
        int check_duplicate_node(double *);
        void print_path(const double*, const double*);
        void close_logfile(void);
        void path_to_closest_goal(void);
        ~RRT();
    private:
        Node initial_node, goal_node;
        double *goal_state, *initial_state;
        int max_nodes;
        RobotModel *veh;
        World *env;
        ofstream fp, controlfp;
        Graph g;
        ArcMapWeight *cost;
        NodeMapState *states;
        ArcMapControl *control_map;
};

RRT::RRT(double *init, double *goal, int n, RobotModel *car, World *w,
         char *fname, char *ctrl_fname)
{
    cout << "Criando instancia da classe RRT" << endl;
    goal_state = (double*) malloc(sizeof(double) * car->n_states);
    initial_state = (double*) malloc(sizeof(double) * car->n_states);
    memcpy(goal_state, goal, sizeof(double) * car->n_states);
    memcpy(initial_state, init, sizeof(double) * car->n_states);
    states = new NodeMapState(g);
    cost = new ArcMapWeight(g);
    control_map = new ArcMapControl(g);
    initial_node = g.addNode();
    (*states)[initial_node] = initial_state;
    max_nodes = n;
    veh = car;
    env = w;
    fp.open(fname);
    controlfp.open(ctrl_fname);
    initiate_rand_number_generator();
}

RRT::~RRT()
{
    cout << "Destruindo instancia da classe RRT" << endl;
}

void RRT::build(void)
{
    int finished=0, i=0;
    double random_state[veh->n_states];
    int prev=0, j=0, collided;
    while(i < max_nodes)
    {
        collided=1;
        j++;
        //Se amostra maior que GOAL_BIAS, ponto aleatorio eh escolhido para expandir
        //arvore. Caso contrario eh escolhido o destino para a expansao.
        if(rnd() > GOAL_BIAS)
        {
            while(collided)
            {
                biased_sampling(env->dim, random_state);
                collided = env->is_vehicle_in_collision(random_state[0], random_state[1], random_state[2]);
            }
        }
        else
        {
            prev++;
            memcpy(random_state, goal_state, sizeof(double) * veh->n_states);
        }
        finished = extend(random_state);
        if(finished == 1)
            i++;
        else if(finished == 2)
        {
            cout << "[" << i << "] Objetivo alcancado." << endl;
            //path_finder();
            //print_nodes(nodes);
            break;
        }
    }
    cout << "Tentativas de adicionar nos: " << i<< " Previlegiou: " << prev << endl;
}

int RRT::extend(double *rand)
{
    Node near = select_nearest_node(rand), added;
    double expanded[veh->n_states], *pt=NULL, *choosed=NULL, *best_control=NULL, temp[veh->n_states];
    bool not_collided;
    /*double u[42][2]={{0.50, 0.00}, {0.50, 0.02}, {0.50, 0.07}, {0.50, 0.12},
                     {0.50, 0.17}, {0.50, 0.23}, {0.50, 0.28}, {0.50, 0.33},
                     {0.50, 0.38}, {0.50, 0.44}, {0.50, 0.49}, {-0.50, 0.00},
                     {-0.50, 0.02}, {-0.50, 0.07}, {-0.50, 0.12}, {-0.50, 0.17},
                     {-0.50, 0.23}, {-0.50, 0.28}, {-0.50, 0.33}, {-0.50, 0.38},
                     {-0.50, 0.44}, {-0.50, 0.49}, {0.50, -0.02}, {0.50, -0.07},
                     {0.50, -0.12}, {0.50, -0.17}, {0.50, -0.23}, {0.50, -0.28},
                     {0.50, -0.33}, {0.50, -0.38}, {0.50, -0.44}, {0.50, -0.49},
                     {-0.50, -0.02}, {-0.50, -0.07}, {-0.50, -0.12},
                     {-0.50, -0.17}, {-0.50, -0.23}, {-0.50, -0.28},
                     {-0.50, -0.33}, {-0.50, -0.38}, {-0.50, -0.44},
                     {-0.50, -0.49}
                    };*/
    double u[3][2] = {{5.0,5.0}, {-5.0, 5.0}, {5.0, -5.0}};
    double d=1e7, aux;
    int duplicated_node_id;
    for(int i=0; i<3; i++)
    {
        not_collided = check_no_collision_path((*states)[near], u[i], expanded);
        if (not_collided)
        {
            aux = metric(expanded, rand);
            if(aux < d)
            {
                d = aux;
                memcpy(temp, expanded, sizeof(double) * veh->n_states);
                pt = temp;
                best_control = u[i];
            }
        }
    }
    if (pt)
    {
        double *arcmap_best_control;
        arcmap_best_control = (double *) malloc(sizeof(double) * 2);
        memcpy(arcmap_best_control, best_control, sizeof(double) * 2);
        duplicated_node_id = check_duplicate_node(temp);
        //cout << "ID: " << duplicated_node_id << " ";
        if (duplicated_node_id >= 0)
        {
            //cout << "Duplicated NODE detect ID: " << duplicated_node_id << endl;
            Node n = g.nodeFromId(duplicated_node_id);
            Graph::Arc arc = g.addArc(near, n);
            (*control_map)[arc] = arcmap_best_control;
            (*cost)[arc] = d;
            return -1;
        }
        print_path((*states)[near], best_control);
        choosed = (double*) malloc(sizeof(double) * veh->n_states);
        memcpy(choosed, temp, sizeof(double) * veh->n_states);
        added = g.addNode();
        (*states)[added] = choosed;
        Graph::Arc arc = g.addArc(near, added);
        (*control_map)[arc] = arcmap_best_control;
        (*cost)[arc] = d;
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
    double it, aux[veh->n_states], temp[veh->n_states], x, y, theta;
    bool in_collision;
    memcpy(aux, near_node, sizeof(double) * veh->n_states);
    for (it=0.0; it<INTEGRATION_TIME; it+=DELTA_T)
    {
        veh->EstimateNewState(DELTA_T, aux, u, temp);
        x = temp[STATE_X];
        y = temp[STATE_Y];
        theta = normalize_angle(temp[STATE_THETA]);
        in_collision = env->is_vehicle_in_collision(x, y, theta);
        if (in_collision)
        {
            //cout << "COLLISION" << endl;
            return false;
        }
        memcpy(aux, temp, sizeof(double) * veh->n_states);
    }
    memcpy(new_state, aux, sizeof(double) * veh->n_states);
    return true;
}


int RRT::check_duplicate_node(double *adding)
{
    double aux;
    for(Graph::NodeIt n(g); n != INVALID; ++n)
    {
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
    for(int i=0; i<veh->n_states; i++)
        fp << near_node[i] << " ";
    fp << endl;
    controlfp << best_control[0] << " " << best_control[1] << " " << INTEGRATION_TIME << endl;
}

void RRT::close_logfile(void)
{
    fp.close();
}

void RRT::path_to_closest_goal(void)
{
    Dijkstra<Graph, ArcMapWeight> ss(g, (*cost));
    Node closest_goal;
    MyPath p;
    double distance;
    ofstream pathfile("path.log");
    closest_goal = select_nearest_node(goal_state);
    cout << (*states)[closest_goal][0] << " " << (*states)[closest_goal][1] << " " << (*states)[closest_goal][2] << endl;
    distance = metric((*states)[closest_goal], goal_state);
    cout << "Distancia entre no mais proximo de goal: " << distance << endl;
    ss.run(initial_node);
    p = ss.path(closest_goal);
    for (MyPath::ArcIt a(p); a != INVALID; ++a)
    {
        Node s = g.source(a);
        Node t = g.target(a);
        cout << "From State: " << (*states)[s][0] <<" "<< (*states)[s][1] <<" " << (*states)[s][2] << endl;
        cout << "To State: " << (*states)[t][0] <<" "<< (*states)[t][1] <<" " << (*states)[t][2] << endl;
        cout << "Control map: " << (*control_map)[a][0] << " " << (*control_map)[a][1] << endl;
        cout << "Arc cost: " << (*cost)[a] << endl;
        for(int i=0; i<veh->n_states; i++)
            pathfile << (*states)[s][i] << " ";
        pathfile << (*control_map)[a][0] << " " << (*control_map)[a][1] << " "
                 << INTEGRATION_TIME << endl;
    }
    pathfile.close();
    //goal_node = g.addNode();
    //(*states)[goal_node] = goal_state;
}
   
#endif
