#ifndef _GRAPHICS_UTIL_H_
#define _GRAPHICS_UTIL_H_

#include <iostream>
#include <fstream>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include "simple_car.h"
#include "utils.h"
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
        void draw(IplImage*, CvScalar);
};


Carro::Carro(ModelCar *car, double x, double y, double theta)
{
    //cout << "Criando instancia da classe Carro." << endl;
    double xrt, yrt, xlt, ylt, xlb, ylb, xrb, yrb, xc, yc, aux_x, aux_y, l, h, w;
    double rot_mat[4];
    w = car->get_body_width();
    h = car->get_body_height();
    l = car->get_between_axes_length();
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
    
    retangulo[0] = cvPoint(SCALE_FACTOR * xlt, SCALE_FACTOR * ylt);
    retangulo[1] = cvPoint(SCALE_FACTOR * xrt, SCALE_FACTOR * yrt);
    retangulo[2] = cvPoint(SCALE_FACTOR * xrb, SCALE_FACTOR * yrb);
    retangulo[3] = cvPoint(SCALE_FACTOR * xlb, SCALE_FACTOR * ylb);
    fig[0] = retangulo;
}

Carro::~Carro()
{
    //cout << "Destruindo instancia da classe Carro." << endl;
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
    cout << "Criando instancia da classe EnvModel" << endl;
    ifstream obs_fp(filename);
    char temp[100], *ps, *nxt;
    double v[9];
    int i;
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
        Graphics(ModelCar *);
        ~Graphics();
        void plot_states(char *, int);
        void plot_obstacles(char *);
        void draw(const double, const double, const double, const double,
                  const double, const int);
        void show(void);
        void draw_initial_and_goal(double *initial, double *goal);
    private:
        ModelCar *veh;
        IplImage* img1;
};

Graphics::Graphics(ModelCar *car)
{
    cout << "Criando instancia da classe Graphics" << endl;
    CvSize s = cvSize(400, 400);
    CvScalar white = colors[c_white];
    img1 = cvCreateImage(s, IPL_DEPTH_32F, 3);
    cvNamedWindow("win1", CV_WINDOW_AUTOSIZE);
    cvRectangle(img1, cvPoint(0,0), cvPoint(400,400), white, -1, 8, 0);
    veh = car;
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
    float x, y, theta, v, phi, t;
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

        v = strtod(nxt, &ps);
        nxt = ps;

        phi = strtod(nxt, &ps);
        nxt = ps;

        t = strtod(nxt, NULL);
        draw(x, y, theta, v, phi, color);
        i += 1;
    }
    cout << i << " pontos lidos." << endl;
    fp.close();
}

void Graphics::draw(const double x, const double y, const double theta,
                    const double v, const double phi, const int color_id)
{
    double aux[3], u[2], temp[3];
    u[0] = v; u[1] = phi;
    aux[0] = x; aux[1] = y; aux[2] = theta;
    Carro car(veh, x, y, theta);
    CvScalar color = colors[color_id];
    car.draw(img1, color);
    veh->EstimateNewState(INTEGRATION_TIME, aux, u, temp);
    Carro carf(veh, temp[0], temp[1], temp[2]);
    carf.draw(img1, color);
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
