#ifndef __GRAPHICS_HH__
#define __GRAPHICS_HH__

#include <fstream>
#include <cv.h>
#include <highgui.h>
#include "Geometry.hh"
#include "RobotModel.hh"
#include "DrawingUtils.hh"

// Forward declaration
class Obstacles;

class Graphics
{
    public:
        Graphics(CarGeometry *, char *, char*);
        ~Graphics();
        void plot_trail_states(char *, int);
        void plot_line_states(char *, int);
        void plot_tree(char *, int, RobotModel *);
        void plot_obstacles(void);
        void draw_trail(double, double, double, int);
        void draw_line(double, double, double, double, int);
        void show(char*);
        void draw_initial_and_goal(double *initial, double *goal);
    private:
        CarGeometry *veh_geom;
        Obstacles *map_enviroment;
        IplImage* img1;
};

#endif
