#ifndef _GRAPHICS_UTIL_H_
#define _GRAPHICS_UTIL_H_

#include <iostream>
#include <fstream>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include "simple_car.h"
#include "world.h"

using namespace std;

class Carro
{
    public:
        CvPoint retangulo[4];
        CvPoint *fig[1];
        Carro(ModelCar*, double, double, double);
        ~Carro();
        void transform(CvPoint, CvPoint, double);
        void draw(IplImage*);
};


Carro::Carro(ModelCar *car, double x, double y, double theta)
{
    //cout << "Criando instancia da classe Carro." << endl;
    double xrt, yrt, xlt, ylt, xlb, ylb, xrb, yrb, xc, yc, aux_x, aux_y, l, h, w;
    double rot_mat[4];
    x *= 10.0;
    y *= 10.0;
    l = 10.0 * car->get_between_axes_length();
    h = 10.0 * car->get_body_width();
    w = 10.0 * car->get_body_height();
    rot_mat[3] = rot_mat[0] = cos(theta);
    rot_mat[2] = sin(theta);
    rot_mat[1] = -rot_mat[2];

    xc=x + l + (w-l)/2.0;
    yc=y + h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrt = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrt = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x - (w-l)/2.0;
    yc=y + h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xlt = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    ylt = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x - (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xlb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    ylb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;

    xc=x + l + (w-l)/2.0;
    yc=y - h/2.0;
    aux_x = xc * rot_mat[0] + yc * rot_mat[1];
    aux_y = xc * rot_mat[2] + yc * rot_mat[3];
    xrb = aux_x + x - rot_mat[0]*x - rot_mat[1]*y;
    yrb = aux_y + y - rot_mat[2]*x - rot_mat[3]*y;
    
    retangulo[0] = cvPoint(xlt,ylt);
    retangulo[1] = cvPoint(xrt,yrt);
    retangulo[2] = cvPoint(xrb,yrb);
    retangulo[3] = cvPoint(xlb,ylb);
    fig[0] = retangulo;
}

Carro::~Carro()
{
    //cout << "Destruindo instancia da classe Carro." << endl;
}

void Carro::draw(IplImage* img)
{
    int npts=4, ncurves = 1;
    CvScalar blue = CV_RGB(0,0,255);
    cvPolyLine(img, fig, &npts, ncurves, 1, blue);
}

class Graphics
{
    public:
        Graphics(char *, char *, ModelCar *, World *);
        ~Graphics();
        void read_and_plot_everything(void);
        void read_and_plot_best_path(void);
        void draw(IplImage *, const double, const double, const double,
                   const double, const double, const double);
        void show(void);
        void read_obstacles(void);
    private:
        ModelCar *veh;
        World *world;
        ifstream resultsfp, pathfp;
        IplImage *everything, *best_path ;
};

Graphics::Graphics(char *resultsfile, char *pathfile, ModelCar *car, World *wrl)
{
    cout << "Criando instancia da classe Graphics" << endl;
    CvSize s = cvSize(400, 400);
    CvScalar white = CV_RGB(255, 255, 255);
    //Obstacles my_env;
    resultsfp.open(resultsfile);
    pathfp.open(pathfile);
    everything = cvCreateImage(s, IPL_DEPTH_32F, 3);
    best_path = cvCreateImage(s, IPL_DEPTH_32F, 3);
    cvNamedWindow("win1", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("win2", CV_WINDOW_AUTOSIZE);
    cvRectangle(everything, cvPoint(0,0), cvPoint(400,400), white, -1, 8, 0);
    cvRectangle(best_path, cvPoint(0,0), cvPoint(400,400), white, -1, 8, 0);
    world = wrl;
    wrl->env->plot_obstacles(everything);
    wrl->env->plot_obstacles(best_path);
    veh = car;
}

Graphics::~Graphics()
{
    cout << "Destruindo instancia da classe Graphics" << endl;
    cvReleaseImage(&everything);
    cvReleaseImage(&best_path);
    resultsfp.close();
    pathfp.close();
}

void Graphics::read_and_plot_everything()
{
    char temp[100], *ps, *nxt;
    int i;
    float x, y, phi, v, w, t;
    cout << "Lendo pontos do arquivo. " << endl;
    i = 0;
    while(resultsfp.getline(temp, 100))
    {
        //cout << temp << endl;
        x = strtod(temp, &ps);
        nxt = ps;
        //cout << "X: " << x << endl;
        
        
        y = strtod(nxt, &ps);
        nxt = ps;
        //cout << "Y: " << y << endl;

        phi = strtod(nxt, &ps);
        nxt = ps;
        //cout << "PHI: " << phi << endl;

        v = strtod(nxt, &ps);
        nxt = ps;
        //cout << "v: " << v << endl;

        w = strtod(nxt, &ps);
        nxt = ps;
        //cout << "W: " << w << endl;

        t = strtod(nxt, NULL);
        //cout << "T: " << t << endl;
        draw(everything, x, y, phi, v, w, t);
        i += 1;
    }
    cout << i << " pontos lidos." << endl;
}

void Graphics::read_and_plot_best_path()
{
    char temp[100], *ps, *nxt;
    int i;
    float x, y, phi, v, w, t;
    cout << "Lendo pontos do arquivo. " << endl;
    i = 0;
    while(pathfp.getline(temp, 100))
    {
        //cout << temp << endl;
        x = strtod(temp, &ps);
        nxt = ps;
        //cout << "X: " << x << endl;
        
        
        y = strtod(nxt, &ps);
        nxt = ps;
        //cout << "Y: " << y << endl;

        phi = strtod(nxt, &ps);
        nxt = ps;
        //cout << "PHI: " << phi << endl;

        v = strtod(nxt, &ps);
        nxt = ps;
        //cout << "v: " << v << endl;

        w = strtod(nxt, &ps);
        nxt = ps;
        //cout << "W: " << w << endl;

        t = strtod(nxt, NULL);
        //cout << "T: " << t << endl;
        draw(best_path, x, y, phi, v, w, t);
        i += 1;
    }
    cout << i << " pontos lidos." << endl;
}

void Graphics::draw(IplImage *img, const double x, const double y,
                    const double phi, const double v, const double w,
                    const double t)
{
    double i, dt=0.05, temp[3], aux[3], u[2];
    Carro car(veh, x, y, phi);
    CvScalar green = CV_RGB(0,255,0);
    CvPoint p0, p1;
    u[0] = v; u[1] = w;
    aux[0] = x; aux[1] = y; aux[2] = phi;
    car.draw(img);
    for (i=0; i<=t; i=i+dt)
    {
        veh->EstimateNewState(DELTA_T, aux, u, temp);
        p0 = cvPoint(aux[0] * 10.0, aux[1] * 10.0);
        p1 = cvPoint(temp[0] * 10.0, temp[1] * 10.0);
        cvLine(img, p0, p1, green, 1, 8, 0);
        memcpy(aux, temp, sizeof(double) * 3);
    }
}


void Graphics::show(void)
{
    cvShowImage("win1", everything);
    cvShowImage("win2", best_path);
    cvWaitKey(0);
}

#endif
