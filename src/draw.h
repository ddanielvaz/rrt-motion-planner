#ifndef _GRAPHICS_UTIL_H_
#define _GRAPHICS_UTIL_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include "geometry.h"
#include "robots.h"
#include "utils.h"
#include "world.h"

using namespace std;

class Carro
{
    public:
        Carro(CarGeometry*);
        void draw(IplImage*, CvScalar);
    private:
        CvPoint retangulo[4];
        CvPoint *fig[1];
};


Carro::Carro(CarGeometry *car)
{
    //cout << "Criando instancia da classe Carro." << endl;
    retangulo[0] = cvPoint(SCALE_FACTOR * car->xlt, SCALE_FACTOR * car->ylt);
    retangulo[1] = cvPoint(SCALE_FACTOR * car->xrt, SCALE_FACTOR * car->yrt);
    retangulo[2] = cvPoint(SCALE_FACTOR * car->xrb, SCALE_FACTOR * car->yrb);
    retangulo[3] = cvPoint(SCALE_FACTOR * car->xlb, SCALE_FACTOR * car->ylb);
    fig[0] = retangulo;
}

void Carro::draw(IplImage* img, CvScalar color)
{
    int npts=4, ncurves = 1;
    cvPolyLine(img, fig, &npts, ncurves, 1, color);
}

class Obstacles
{
    public:
        Obstacles(char *);
        void draw(IplImage *);
        vector<CvPoint3D64f> triangs;
};

Obstacles::Obstacles(char *filename)
{
    cout << "Criando instancia da classe Obstacles" << endl;
    ifstream obs_fp(filename);
    char temp[100], *ps, *nxt;
    double v[9];
    int i;
    // Descartando primeira linha de informação sobre dimensão do ambiente.
    obs_fp.getline(temp, 100);
    while(obs_fp.getline(temp, 100))
    {
        v[0] = strtod(temp, &ps);
        nxt = ps;
        for (i=1; i<9; i++)
        {
            v[i] = strtod(nxt, &ps);
            nxt = ps;
        }
        for(i=0;i<9;i=i+3)
            triangs.push_back(cvPoint3D64f(v[i], v[i+1], v[i+2]));
    }
    obs_fp.close();
}

void Obstacles::draw(IplImage *img)
{
    CvPoint triang[3];
    CvPoint *fig[1];
    int npts=3, ncurves=1;
    CvScalar red = CV_RGB(255,0,0);
    for (vector<CvPoint3D64f>::iterator i = triangs.begin(); i != triangs.end(); ){
        triang[0] = cvPoint(SCALE_FACTOR * i->x, SCALE_FACTOR * i->y);
        ++i;
        triang[1] = cvPoint(SCALE_FACTOR * i->x, SCALE_FACTOR * i->y);
        ++i;
        triang[2] = cvPoint(SCALE_FACTOR * i->x, SCALE_FACTOR * i->y);
        ++i;
        fig[0] = triang;
        cvPolyLine(img, fig, &npts, ncurves, 1, red);
    }
}

class Graphics
{
    public:
        Graphics(CarGeometry *);
        ~Graphics();
        void plot_states(char *, int);
        void plot_obstacles(char *);
        void draw(double, double, double, int);
        void show(void);
        void draw_initial_and_goal(double *initial, double *goal);
    private:
        CarGeometry *veh_geom;
        IplImage* img1;
};

Graphics::Graphics(CarGeometry *car_geom)
{
    cout << "Criando instancia da classe Graphics" << endl;
    CvSize s = cvSize(410, 410);
    CvScalar white = colors[c_white];
    img1 = cvCreateImage(s, IPL_DEPTH_32F, 3);
    cvNamedWindow("win1", CV_WINDOW_AUTOSIZE);
    cvRectangle(img1, cvPoint(0,0), cvPoint(410,410), white, -1, 8, 0);
    veh_geom = car_geom;
}

Graphics::~Graphics()
{
    cout << "Destruindo instancia da classe Graphics" << endl;
    cvReleaseImage(&img1);
}

void Graphics::draw_initial_and_goal(double *initial, double *goal)
{
    CvScalar green = colors[c_green], red = colors[c_red];
    double x, y, r=1.0;
    x = SCALE_FACTOR * initial[0];
    y = SCALE_FACTOR * initial[1];
    cvCircle(img1, cvPoint(x,y), r, red, -1, 8, 0);
    x = SCALE_FACTOR * goal[0];
    y = SCALE_FACTOR * goal[1];
    cvCircle(img1, cvPoint(x,y), r, green, -1, 8, 0);
}

void Graphics::plot_states(char *filename, int color)
{
    char temp[100], *ps, *nxt;
    int i;
    float x, y, theta;
    ifstream fp(filename);
    cout << "Lendo pontos do arquivo. " << endl;
    i = 0;
    while(fp.getline(temp, 100))
    {
        x = strtod(temp, &ps);
        nxt = ps;

        y = strtod(nxt, &ps);
        nxt = ps;

        theta = strtod(nxt, &ps);
        nxt = ps;
        draw(x, y, theta, color);
        i++;
    }
    cout << i << " pontos lidos." << endl;
    fp.close();
}

void Graphics::draw(double x, double y, double theta, int color_id)
{
    cvCircle(img1, cvPoint(x * SCALE_FACTOR,y * SCALE_FACTOR), 1.0, colors[c_red], -1, 8, 0);
    veh_geom->position(x, y, theta);
    Carro car(veh_geom);
    CvScalar color = colors[color_id];
    car.draw(img1, color);
}

void Graphics::plot_obstacles(char *filename)
{
    Obstacles obs(filename);
    obs.draw(img1);
}

void Graphics::show(void)
{
    cvShowImage("win1", img1);
    cvWaitKey(0);
}

#endif
