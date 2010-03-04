#ifndef _GRAPHICS_UTIL_H_
#define _GRAPHICS_UTIL_H_

#include <iostream>
#include <fstream>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include "simple_car.h"

using namespace std;

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
    CvSize s = cvSize(200,200);
    fp.open(filename);
    img1 = cvCreateImage(s, IPL_DEPTH_32F, 3);
    cvNamedWindow("win1", CV_WINDOW_AUTOSIZE);
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
        cout << temp << endl;
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
    u[0] = v; u[1] = w;
    aux[0] = x; aux[1] = y; aux[2] = phi;
    for (i=0; i<=t; i=i+dt)
    {
        veh->EstimateNewState(DELTA_T, aux, u, temp);
        cvLine(img, cvPoint(aux[0]*10.0,aux[1]*10.0), cvPoint(temp[0]*10.0,temp[1]*10.0), CV_RGB(255,0,0), 1, 8, 0);
        memcpy(aux, temp, sizeof(double) * 3);
    }
    //cvRectangle(img, cvPoint(x-1,y-1), cvPoint(x+1,y+1),CV_RGB(0,255,0), 1, 8, 0);
    //cvRectangle(img, cvPoint(xf-1,yf-1), cvPoint(xf+1,yf+1),CV_RGB(0,255,0), 1, 8, 0);
    //cvWaitKey(0);
}

void Graphics::show(void)
{
    cvShowImage("win1", img1);
    // wait for a key
    cvWaitKey(0);
}

#endif
