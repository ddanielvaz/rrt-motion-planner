#include "RRT.hh"

RRT::RRT(double *init, double *goal, int n, RobotModel *car, World *w,
         DistanceMeter *d_meter, StateSampler *s_sampler, char *fname)
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
    trial_max = n;
    veh = car;
    world = w;
    meter = d_meter;
    sampler = s_sampler;
    fp.open(fname);
    added_nodes = 0;
    trials = 0;
    is_goal_reached = false;
}

RRT::~RRT()
{
    cout << "Destruindo instancia da classe RRT" << endl;
}

void RRT::build(void)
{
    int finished=0;
    double random_state[veh->n_states];
    int previleges_qgoal=0, collided;
    for(trials=0; trials<trial_max; trials++)
    {
        //Se amostra maior que GOAL_BIAS, ponto aleatorio eh escolhido para expandir
        //arvore. Caso contrario eh escolhido o destino para a expansao.
        if(rnd() > GOAL_BIAS)
        {
            collided=1;
            while(collided)
            {
                if(rnd() <= AVG_BIAS)
                {
                    sampler->AvgBiasedSampling(world->env->dim, random_state);
                    collided = world->IsVehicleInCollision(random_state[0], random_state[1], random_state[2]);
                }
                else
                {
                    sampler->BiasedSampling(world->env->dim, random_state);
                    collided = world->IsVehicleInCollision(random_state[0], random_state[1], random_state[2]);
                }
            }
        }
        else
        {
            previleges_qgoal++;
            memcpy(random_state, goal_state, sizeof(double) * veh->n_states);
        }
        finished = extend(random_state);
        if(finished == 1)
            added_nodes++;
        else if(finished == 2)
        {
            cout << "[" << trials << "] Objetivo alcancado." << endl;
            is_goal_reached = true;
            break;
        }
    }
    cout << "Numero de tentativas: Maxima= " << trial_max << " Realizadas= " << trials << endl;
    cout << "Nos adicionados: " << added_nodes << endl;
    cout << "Previlegiou q_goal: " << previleges_qgoal << endl;
}

int RRT::extend(double *rand)
{
    Node near = select_nearest_node(rand), added;
    double expanded[veh->n_states], *pt=NULL, *choosed=NULL, *best_control=NULL, temp[veh->n_states];
    bool not_collided;
    double d=1e7, aux;
    int duplicated_node_id;
    vector<control_input>::iterator it;
    veh->GetValidInputs((*states)[near]);
    for ( it=veh->inputs.begin() ; it < veh->inputs.end(); it++ )
    {
        not_collided = check_no_collision_path((*states)[near], (*it).ctrl, expanded);
        if (not_collided)
        {
            aux = meter->DistanceWeight(expanded, rand);
            if(aux < d)
            {
                d = aux;
                memcpy(temp, expanded, sizeof(double) * veh->n_states);
                pt = temp;
                best_control = (*it).ctrl;
            }
        }
    }
    if (pt)
    {
        double *arcmap_best_control;
        arcmap_best_control = (double *) malloc(sizeof(double) * 2);
        memcpy(arcmap_best_control, best_control, sizeof(double) * 2);
        duplicated_node_id = check_duplicate_node(temp);
//         cout << "ID: " << duplicated_node_id << " ";
        if (duplicated_node_id >= 0)
        {
//             cout << "Duplicated NODE detect ID: " << duplicated_node_id << endl;
            Node n = g.nodeFromId(duplicated_node_id);
            Graph::Arc arc = g.addArc(near, n);
            (*control_map)[arc] = arcmap_best_control;
            (*cost)[arc] = d;
            return -1;
        }
        export_path((*states)[near], best_control);
        choosed = (double*) malloc(sizeof(double) * veh->n_states);
        memcpy(choosed, temp, sizeof(double) * veh->n_states);
        added = g.addNode();
        (*states)[added] = choosed;
        Graph::Arc arc = g.addArc(near, added);
        (*control_map)[arc] = arcmap_best_control;
        (*cost)[arc] = d;
        if(meter->GoalStateAchieved((*states)[added], goal_state))
            return 2;
        else
            return 1;
    }
//     cout << "sem no para adicionar" << endl;
    return -1;
}

Node RRT::select_nearest_node(double *rand)
{
    double min_distance = 1e7, aux;
    Node nearest;
    for(Graph::NodeIt n(g); n != INVALID; ++n)
    {
        aux = meter->NearestNodeMetric((*states)[n], rand);
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
    double t, aux[veh->n_states], temp[veh->n_states], x, y, theta;
    bool in_collision;
    memcpy(aux, near_node, sizeof(double) * veh->n_states);
    for (t=0.0; t<INTEGRATION_TIME; t+=DELTA_T)
    {
        veh->EstimateNewState(aux, u, temp);
        x = temp[STATE_X];
        y = temp[STATE_Y];
        theta = normalize_angle(temp[STATE_THETA]);
        temp[STATE_THETA] = theta;
        in_collision = world->IsVehicleInCollision(x, y, theta);
        if (in_collision)
        {
//             cout << "COLLISION" << endl;
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
        aux = meter->DistanceWeight((*states)[n], adding);
        if (aux < 1e-7)
        {
            return g.id(n);
        }
    }
    return -1;
}

void RRT::export_path(const double *near_node, const double *best_control)
{
    for(int i=0; i<veh->n_states; i++)
        fp << near_node[i] << " ";
    fp << best_control[0] << " " << best_control[1] << " " << INTEGRATION_TIME << endl;
}

void RRT::close_logfile(void)
{
    fp.close();
}

void RRT::path_to_closest_goal(char *filename)
{
    Dijkstra<Graph, ArcMapWeight> ss(g, (*cost));
    Node closest_goal;
    MyPath p;
    double distance;
    ofstream pathfile(filename);
    closest_goal = select_nearest_node(goal_state);
    cout << (*states)[closest_goal][0] << " " << (*states)[closest_goal][1] << " " << (*states)[closest_goal][2] << endl;
    distance = meter->DistanceWeight((*states)[closest_goal], goal_state);
    cout << "Distancia entre no mais proximo de goal: " << distance << endl << endl;
    ss.run(initial_node);
    p = ss.path(closest_goal);
//     for(int i=0; i<veh->n_states; i++)
//             pathfile << (*states)[initial_node][i] << " ";
//     pathfile << "0.0 0.0 " << INTEGRATION_TIME << endl;
    for (MyPath::ArcIt a(p); a != INVALID; ++a)
    {
        Node s = g.source(a);
        Node t = g.target(a);
        /*cout << "From State: " << (*states)[s][0] <<" "<< (*states)[s][1] <<" " << (*states)[s][2] << endl;
        cout << "To State: " << (*states)[t][0] <<" "<< (*states)[t][1] <<" " << (*states)[t][2] << endl;
        cout << "Control map: " << (*control_map)[a][0] << " " << (*control_map)[a][1] << endl;
        cout << "Arc cost: " << (*cost)[a] << endl;*/
        for(int i=0; i<veh->n_states; i++)
            pathfile << (*states)[s][i] << " ";
        pathfile << (*control_map)[a][0] << " " << (*control_map)[a][1] << " "
                 << INTEGRATION_TIME << endl;
    }
    pathfile.close();
    //goal_node = g.addNode();
    //(*states)[goal_node] = goal_state;
}
