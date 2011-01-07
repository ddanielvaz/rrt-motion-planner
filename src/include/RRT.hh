#ifndef __Rapidly_Random_Tree_HH__
#define __Rapidly_Random_Tree_HH__

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>
#include <lemon/random.h>

#include "DistanceMeter.hh"
#include "StateSampler.hh"
#include "RobotModel.hh"
#include "World.hh"

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <cstring>

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
        RRT(double*, double*, int, RobotModel*, World*, DistanceMeter*,
            StateSampler*, char*);
        void build(void);
        int extend(double*);
        Node select_nearest_node(double*);
        bool check_no_collision_path(const double*, const double*, double*);
        int check_duplicate_node(double *);
        void export_path(const double*, const double*);
        void close_logfile(void);
        void path_to_closest_goal(char*);
        ~RRT();
        int added_nodes, trials;
        bool is_goal_reached;
    private:
        Node initial_node, goal_node;
        double *goal_state, *initial_state;
        int trial_max;
        RobotModel *veh;
        World *world;
        DistanceMeter *meter;
        StateSampler *sampler;
        ofstream fp;
        Graph g;
        ArcMapWeight *cost;
        NodeMapState *states;
        ArcMapControl *control_map;
};

#endif
