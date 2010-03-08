#ifndef _GRAPHICS_UTIL_H_
#define _GRAPHICS_UTIL_H_

#include <iostream>
#include <fstream>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include "simple_car.h"

using namespace std;

/*
L distancia entre os eixos
H comprimento total
W largura do carro
*/

class Carro
{
    private:
        double L, H, W;
        double xrt, yrt, xlt, ylt, xlb, ylb, xrb, yrb, xo, yo;
        CvPoint initial;
    public:
        CvPoint retangulo[4];
        CvPoint *fig[1];
        Carro(double, double, double);
        ~Carro();
        void transform(CvPoint, CvPoint, double);
        void draw(IplImage*);
};

//Carro::Carro(double width, double height, double axes_len, double x, double y,double theta)

Carro::Carro(double x, double y, double theta)
{
    //cout << "Criando instancia da classe Carro." << endl;
    W = 20.0;
    H = 10.0;
    L = 15.0;
    xo = x;
    yo = y;
    initial = cvPoint(xo, yo);
    xrt=xo + L + (W-L)/2.0;
    yrt=yo + H/2.0;
    
    xlt=xo - (W-L)/2.0;
    ylt=yrt;
    
    xlb=xlt;
    ylb=yo - H/2.0;
    
    xrb=xrt;
    yrb=ylb;
    retangulo[0] = cvPoint(xlt,ylt);
    retangulo[1] = cvPoint(xrt,yrt);
    retangulo[2] = cvPoint(xrb,yrb);
    retangulo[3] = cvPoint(xlb,ylb);
    fig[0] = retangulo;
    transform(initial, cvPoint(0,0), theta);
}

Carro::~Carro()
{
    //cout << "Destruindo instancia da classe Carro." << endl;
}

void Carro::transform(CvPoint prot, CvPoint curr, double angle)
{
    double rot_mat[4] = {cos(angle), -sin(angle), sin(angle), cos(angle)};
    double aux_x, aux_y;
    int i=0;
    for(i=0; i<4; i++)
    {
        aux_x = retangulo[i].x * rot_mat[0] + retangulo[i].y * rot_mat[1];
        aux_y = retangulo[i].x * rot_mat[2] + retangulo[i].y * rot_mat[3];
        retangulo[i].x = aux_x + prot.x - rot_mat[0]*prot.x - rot_mat[1]*prot.y;
        retangulo[i].y = aux_y + prot.y - rot_mat[2]*prot.x - rot_mat[3]*prot.y;
    }
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
        Graphics(char *filename, ModelCar*);
        ~Graphics();
        void plot(void);
        void draw(IplImage *, const double, const double, const double,
                   const double, const double, const double);
        void show(void);
    private:
        ModelCar *veh;
        ifstream fp;
        IplImage* img1;
};

Graphics::Graphics(char *filename, ModelCar *car)
{
    CvSize s = cvSize(400, 400);
    CvScalar white = CV_RGB(255, 255, 255);
    fp.open(filename);
    img1 = cvCreateImage(s, IPL_DEPTH_32F, 3);
    cvNamedWindow("win1", CV_WINDOW_AUTOSIZE);
    cvRectangle(img1, cvPoint(0,0), cvPoint(400,400), white, -1, 8, 0);
    veh = car;
}

Graphics::~Graphics()
{
    cout << "Destruindo instancia da classe Graphics" << endl;
    cvReleaseImage(&img1);
    fp.close();
}

void Graphics::plot()
{
    char temp[100], *ps, *nxt;
    int i;
    float x, y, phi, v, w, t;
    cout << "Lendo pontos do arquivo. " << endl;
    i = 0;
    while(fp.getline(temp, 100))
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
        draw(img1, x, y, phi, v, w, t);
        i += 1;
    }
    cout << i << " pontos lidos." << endl;
}

void Graphics::draw(IplImage *img, const double x, const double y,
                    const double phi, const double v, const double w,
                    const double t)
{
    double i, dt=0.05, temp[3], aux[3], u[2];
    Carro car(10*x, 10*y, phi);
    u[0] = v; u[1] = w;
    aux[0] = x; aux[1] = y; aux[2] = phi;
    
    car.draw(img);
    for (i=0; i<=t; i=i+dt)
    {
        veh->EstimateNewState(DELTA_T, aux, u, temp);
        cvLine(img, cvPoint(aux[0]*10.0,aux[1]*10.0), cvPoint(temp[0]*10.0,temp[1]*10.0), CV_RGB(255,0,0), 1, 8, 0);
        memcpy(aux, temp, sizeof(double) * 3);
    }
}

void Graphics::show(void)
{
    cvShowImage("win1", img1);
    cvWaitKey(0);
}

#endif
